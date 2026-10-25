using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using bloom.Services;
using Microsoft.Extensions.Configuration;
using Moq;
using Xunit;

namespace bloom.Tests.Services;

public class StudyBoxUploaderTests : IDisposable
{
    private readonly string _directory = Path.Combine(Path.GetTempPath(), "study-box-" + Guid.NewGuid());
    private StudyBoxUploader Service(Handler handler, bool configured = true)
    {
        var settings = new Dictionary<string, string?>();
        if (configured) foreach (var key in new[] { "FolderId", "ClientId", "ClientSecret", "SubjectId" }) settings["RsrStudy:Box:" + key] = "123";
        var factory = new Mock<IHttpClientFactory>();
        factory.Setup(f => f.CreateClient("StudyBox")).Returns(() => new HttpClient(handler, disposeHandler: false));
        Directory.CreateDirectory(_directory);
        File.WriteAllText(Path.Combine(_directory, "audio"), "full interaction");
        return new StudyBoxUploader(factory.Object, new ConfigurationBuilder().AddInMemoryCollection(settings).Build());
    }
    [Fact]
    public async Task UploadsOnlyOnRequestAndRetriesUsePersistedReceipt()
    {
        var handler = new Handler(); var service = Service(handler);
        Assert.Equal(0, handler.Calls);
        var first = await service.Upload(_directory, "study.webm", CancellationToken.None);
        var second = await service.Upload(_directory, "study.webm", CancellationToken.None);
        Assert.Equal(first, second);
        Assert.Equal("https://app.box.com/file/987", first.Url);
        Assert.Equal(2, handler.Calls);
        Assert.Contains("full interaction", handler.UploadBody);
        Assert.Contains("Bearer token", handler.Authorization);
    }
    [Fact]
    public async Task FailureRetainsAudioAndDoesNotMarkUploaded()
    {
        var handler = new Handler { FailUpload = true }; var service = Service(handler);
        await Assert.ThrowsAsync<HttpRequestException>(() => service.Upload(_directory, "study.webm", CancellationToken.None));
        Assert.Equal("full interaction", File.ReadAllText(Path.Combine(_directory, "audio")));
        Assert.False(File.Exists(Path.Combine(_directory, "box-upload.json")));
    }
    [Fact]
    public async Task MissingConfigurationMakesNoExternalRequest()
    {
        var handler = new Handler(); var service = Service(handler, configured: false);
        await Assert.ThrowsAsync<InvalidOperationException>(() => service.Upload(_directory, "study.webm", CancellationToken.None));
        Assert.Equal(0, handler.Calls);
    }
    [Fact]
    public async Task LargeRecordingUsesChunkUploadAndCommit()
    {
        var handler = new Handler { ChunkUpload = true }; var service = Service(handler);
        using (var file = File.OpenWrite(Path.Combine(_directory, "audio"))) file.SetLength(50_000_001);
        var receipt = await service.Upload(_directory, "study.webm", CancellationToken.None);
        Assert.Equal("987", receipt.FileId);
        Assert.Equal(2, handler.Parts);
        Assert.True(handler.Committed);
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task LostResponseRetryChecksContentBeforeReusingExistingFile(bool different)
    {
        var handler = new Handler { Conflict = true, Different = different }; var service = Service(handler);
        if (different)
        {
            await Assert.ThrowsAsync<InvalidOperationException>(() => service.Upload(_directory, "study.webm", CancellationToken.None));
            Assert.False(File.Exists(Path.Combine(_directory, "box-upload.json")));
        }
        else Assert.Equal("987", (await service.Upload(_directory, "study.webm", CancellationToken.None)).FileId);
    }
    private class Handler : HttpMessageHandler
    {
        public int Calls; public bool FailUpload; public string? UploadBody; public string? Authorization;
        public bool ChunkUpload; public int Parts; public bool Committed;
        public bool Conflict; public bool Different;
        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken ct)
        {
            Calls++;
            if (request.Method == HttpMethod.Get)
            {
                var sha1 = Different ? "different" : Convert.ToHexString(System.Security.Cryptography.SHA1.HashData(System.Text.Encoding.UTF8.GetBytes("full interaction")));
                return new(HttpStatusCode.OK) { Content = new StringContent(System.Text.Json.JsonSerializer.Serialize(new {
                    name = "study.webm", size = 16, sha1, parent = new { id = "123" }
                })) };
            }
            if (request.RequestUri!.Host == "api.box.com") return new(HttpStatusCode.OK) { Content = new StringContent("{\"access_token\":\"token\"}") };
            if (Conflict) return new(HttpStatusCode.Conflict) { Content = new StringContent("{\"context_info\":{\"conflicts\":{\"id\":\"987\"}}}") };
            if (ChunkUpload)
            {
                if (request.Method == HttpMethod.Put)
                {
                    Assert.True(request.Headers.Contains("Digest"));
                    Assert.NotNull(request.Content!.Headers.ContentRange);
                    Parts++;
                    return new(HttpStatusCode.OK) { Content = new StringContent("{\"part\":{\"part_id\":\"p" + Parts + "\",\"offset\":0,\"size\":1,\"sha1\":\"hash\"}}") };
                }
                if (request.RequestUri.AbsolutePath.EndsWith("/commit"))
                {
                    Committed = true;
                    Assert.True(request.Headers.Contains("Digest"));
                    return new(HttpStatusCode.Created) { Content = new StringContent("{\"entries\":[{\"id\":\"987\"}]}") };
                }
                return new(HttpStatusCode.Created) { Content = new StringContent("{\"id\":\"session\",\"part_size\":30000000}") };
            }
            UploadBody = await request.Content!.ReadAsStringAsync(ct); Authorization = request.Headers.Authorization?.ToString();
            return new(FailUpload ? HttpStatusCode.Forbidden : HttpStatusCode.Created) {
                Content = new StringContent("{\"entries\":[{\"id\":\"987\"}]}") };
        }
    }
    public void Dispose() { if (Directory.Exists(_directory)) Directory.Delete(_directory, true); }
}
