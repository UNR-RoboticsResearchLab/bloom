using System.Net;
using System.Net.Http.Headers;
using System.Security.Cryptography;
using System.Text.Json;

namespace bloom.Services;

public record StudyBoxReceipt(string FileId, string Url, string FolderId, DateTime UploadedAt);

/// <summary>Copies immutable archive audio to a server-configured Box folder.</summary>
public sealed class StudyBoxUploader(IHttpClientFactory clients, IConfiguration config)
{
    private readonly SemaphoreSlim _gate = new(1, 1);
    public bool Configured => new[] { "FolderId", "ClientId", "ClientSecret", "SubjectId" }
        .All(key => !string.IsNullOrWhiteSpace(config[$"RsrStudy:Box:{key}"]));

    public async Task<StudyBoxReceipt> Upload(string directory, string name, CancellationToken ct)
    {
        if (!Configured) throw new InvalidOperationException("Box uploads are not configured. Ask the study administrator to configure the destination folder and Box app.");
        await _gate.WaitAsync(ct);
        try
        {
            var receiptPath = Path.Combine(directory, "box-upload.json");
            if (File.Exists(receiptPath))
                return JsonSerializer.Deserialize<StudyBoxReceipt>(await File.ReadAllTextAsync(receiptPath, ct))!;
            using var client = clients.CreateClient("StudyBox");
            using var tokenResponse = await client.PostAsync("https://api.box.com/oauth2/token", new FormUrlEncodedContent(new Dictionary<string, string> {
                ["grant_type"] = "client_credentials", ["client_id"] = config["RsrStudy:Box:ClientId"]!,
                ["client_secret"] = config["RsrStudy:Box:ClientSecret"]!,
                ["box_subject_type"] = config["RsrStudy:Box:SubjectType"] ?? "enterprise",
                ["box_subject_id"] = config["RsrStudy:Box:SubjectId"]!
            }), ct);
            await Check(tokenResponse);
            using var tokenJson = JsonDocument.Parse(await tokenResponse.Content.ReadAsStringAsync(ct));
            client.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", tokenJson.RootElement.GetProperty("access_token").GetString());
            var folder = config["RsrStudy:Box:FolderId"]!;
            await using var audio = File.OpenRead(Path.Combine(directory, "audio"));
            var hash = await SHA1.HashDataAsync(audio, ct);
            audio.Position = 0;
            // Box direct uploads support at most 50 MB; larger archives use parts.
            using var response = audio.Length <= 50_000_000
                ? await Direct(client, audio, folder, name, ct)
                : await Chunked(client, audio, folder, name, hash, ct);
            string id;
            if (response.StatusCode == HttpStatusCode.Conflict)
            {
                // A previous upload may have succeeded before the client disconnected.
                // Reuse it only after checking its bytes, never overwrite a Box file.
                using var conflict = JsonDocument.Parse(await response.Content.ReadAsStringAsync(ct));
                var item = conflict.RootElement.GetProperty("context_info").GetProperty("conflicts");
                if (item.ValueKind == JsonValueKind.Array) item = item[0];
                id = item.GetProperty("id").GetString()!;
                using var existing = await client.GetAsync($"https://api.box.com/2.0/files/{Uri.EscapeDataString(id)}?fields=sha1,size,name,parent", ct);
                await Check(existing);
                using var info = JsonDocument.Parse(await existing.Content.ReadAsStringAsync(ct));
                var file = info.RootElement;
                if (file.GetProperty("name").GetString() != name || file.GetProperty("size").GetInt64() != audio.Length ||
                    file.GetProperty("parent").GetProperty("id").GetString() != folder ||
                    !string.Equals(file.GetProperty("sha1").GetString(), Convert.ToHexString(hash), StringComparison.OrdinalIgnoreCase))
                    throw new InvalidOperationException("A different file already uses this Box filename. No file was overwritten.");
            }
            else
            {
                await Check(response);
                using var result = JsonDocument.Parse(await response.Content.ReadAsStringAsync(ct));
                id = result.RootElement.GetProperty("entries")[0].GetProperty("id").GetString()!;
            }
            var receipt = new StudyBoxReceipt(id, $"https://app.box.com/file/{Uri.EscapeDataString(id)}", folder, DateTime.UtcNow);
            var pending = receiptPath + ".pending";
            await File.WriteAllTextAsync(pending, JsonSerializer.Serialize(receipt), ct);
            File.Move(pending, receiptPath, overwrite: true);
            return receipt;
        }
        finally { _gate.Release(); }
    }

    private static async Task<HttpResponseMessage> Direct(HttpClient client, Stream audio, string folder, string name, CancellationToken ct)
    {
        using var body = new MultipartFormDataContent();
        body.Add(new StringContent(JsonSerializer.Serialize(new { name, parent = new { id = folder } })), "attributes");
        // Keep the archive stream open for duplicate detection after sending.
        body.Add(new StreamContent(new LeaveOpenStream(audio)), "file", name);
        return await client.PostAsync("https://upload.box.com/api/2.0/files/content", body, ct);
    }

    private static async Task<HttpResponseMessage> Chunked(HttpClient client, Stream audio, string folder, string name, byte[] hash, CancellationToken ct)
    {
        using var create = await client.PostAsJsonAsync("https://upload.box.com/api/2.0/files/upload_sessions",
            new { folder_id = folder, file_size = audio.Length, file_name = name }, ct);
        if (create.StatusCode == HttpStatusCode.Conflict)
            return new HttpResponseMessage(HttpStatusCode.Conflict) { Content = new StringContent(await create.Content.ReadAsStringAsync(ct)) };
        await Check(create);
        using var session = JsonDocument.Parse(await create.Content.ReadAsStringAsync(ct));
        var sessionId = Uri.EscapeDataString(session.RootElement.GetProperty("id").GetString()!);
        var url = $"https://upload.box.com/api/2.0/files/upload_sessions/{sessionId}";
        var partSize = session.RootElement.GetProperty("part_size").GetInt32();
        if (partSize <= 0 || partSize > 100_000_000) throw new InvalidOperationException("Box returned an invalid upload part size.");
        var parts = new List<JsonElement>();
        var buffer = new byte[partSize];
        for (long offset = 0; offset < audio.Length;)
        {
            var count = (int)Math.Min(partSize, audio.Length - offset);
            await audio.ReadExactlyAsync(buffer.AsMemory(0, count), ct);
            using var request = new HttpRequestMessage(HttpMethod.Put, url) { Content = new ByteArrayContent(buffer, 0, count) };
            request.Content.Headers.ContentType = new MediaTypeHeaderValue("application/octet-stream");
            request.Content.Headers.ContentRange = new ContentRangeHeaderValue(offset, offset + count - 1, audio.Length);
            request.Headers.TryAddWithoutValidation("Digest", "sha=" + Convert.ToBase64String(SHA1.HashData(buffer.AsSpan(0, count))));
            using var part = await client.SendAsync(request, ct);
            await Check(part);
            using var data = JsonDocument.Parse(await part.Content.ReadAsStringAsync(ct));
            parts.Add(data.RootElement.GetProperty("part").Clone());
            offset += count;
        }
        for (var attempt = 0; attempt < 10; attempt++)
        {
            using var commit = new HttpRequestMessage(HttpMethod.Post, url + "/commit") { Content = JsonContent.Create(new { parts }) };
            commit.Headers.TryAddWithoutValidation("Digest", "sha=" + Convert.ToBase64String(hash));
            var response = await client.SendAsync(commit, ct);
            if (response.StatusCode != HttpStatusCode.Accepted) return response;
            var delay = response.Headers.RetryAfter?.Delta ?? TimeSpan.FromSeconds(2);
            response.Dispose();
            await Task.Delay(TimeSpan.FromSeconds(Math.Clamp(delay.TotalSeconds, 1, 30)), ct);
        }
        throw new InvalidOperationException("Box is still processing the recording. The Bloom archive is retained; retry the upload later.");
    }

    private static Task Check(HttpResponseMessage response)
    {
        if (!response.IsSuccessStatusCode)
            throw new HttpRequestException($"Box returned HTTP {(int)response.StatusCode}. Your Bloom recording is retained; check Box access or retry.");
        return Task.CompletedTask;
    }

    private sealed class LeaveOpenStream(Stream inner) : Stream
    {
        public override bool CanRead => inner.CanRead;
        public override bool CanSeek => inner.CanSeek;
        public override bool CanWrite => false;
        public override long Length => inner.Length;
        public override long Position { get => inner.Position; set => inner.Position = value; }
        public override int Read(byte[] buffer, int offset, int count) => inner.Read(buffer, offset, count);
        public override ValueTask<int> ReadAsync(Memory<byte> buffer, CancellationToken ct = default) => inner.ReadAsync(buffer, ct);
        public override long Seek(long offset, SeekOrigin origin) => inner.Seek(offset, origin);
        public override void Flush() { }
        public override void SetLength(long value) => throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();
    }
}
