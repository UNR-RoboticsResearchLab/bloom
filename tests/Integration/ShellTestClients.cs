using System;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Net.Http.Json;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Mvc.Testing;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Test-side ends of the remote shell's two WebSocket legs, over TestServer's in-process
    /// WebSocket client. Every receive has a timeout so a missing message fails the test instead of
    /// hanging it.
    /// </summary>
    internal sealed class TestShellSocket : IAsyncDisposable
    {
        public static readonly TimeSpan DefaultTimeout = TimeSpan.FromSeconds(5);

        private readonly WebSocket _ws;

        public TestShellSocket(WebSocket ws)
        {
            _ws = ws;
        }

        public WebSocketState State => _ws.State;

        public Task SendJsonAsync(object message) =>
            _ws.SendAsync(JsonSerializer.SerializeToUtf8Bytes(message), WebSocketMessageType.Text, true, CancellationToken.None);

        public Task SendBinaryAsync(byte[] data) =>
            _ws.SendAsync(data, WebSocketMessageType.Binary, true, CancellationToken.None);

        /// <summary>Next message, or null if the server closed the socket.</summary>
        public async Task<(WebSocketMessageType Type, byte[] Data)?> ReceiveAsync(TimeSpan? timeout = null)
        {
            using var cts = new CancellationTokenSource(timeout ?? DefaultTimeout);
            var buffer = new byte[64 * 1024];
            using var message = new MemoryStream();
            while (true)
            {
                WebSocketReceiveResult result;
                try
                {
                    result = await _ws.ReceiveAsync(buffer, cts.Token);
                }
                catch (WebSocketException)
                {
                    return null;
                }
                if (result.MessageType == WebSocketMessageType.Close)
                    return null;
                message.Write(buffer, 0, result.Count);
                if (result.EndOfMessage)
                    return (result.MessageType, message.ToArray());
            }
        }

        public async Task<JsonElement> ReceiveJsonAsync(TimeSpan? timeout = null)
        {
            var message = await ReceiveAsync(timeout)
                ?? throw new InvalidOperationException("Socket closed while waiting for a JSON message.");
            if (message.Type != WebSocketMessageType.Text)
                throw new InvalidOperationException("Expected a text message, got binary.");
            return JsonDocument.Parse(message.Data).RootElement.Clone();
        }

        /// <summary>Reads until the server closes; returns the last JSON message seen, if any.</summary>
        public async Task<JsonElement?> ReceiveUntilClosedAsync(TimeSpan? timeout = null)
        {
            JsonElement? last = null;
            while (await ReceiveAsync(timeout) is { } message)
            {
                if (message.Type == WebSocketMessageType.Text)
                    last = JsonDocument.Parse(message.Data).RootElement.Clone();
            }
            return last;
        }

        public async ValueTask DisposeAsync()
        {
            if (_ws.State is WebSocketState.Open or WebSocketState.CloseReceived)
            {
                try
                {
                    using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(2));
                    await _ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "test done", cts.Token);
                }
                catch (Exception ex) when (ex is WebSocketException or OperationCanceledException)
                {
                }
            }
            _ws.Dispose();
        }
    }

    internal static class ShellTestClients
    {
        public const string SameOrigin = "http://localhost";

        public static async Task<TestShellSocket> ConnectAgentAsync(
            BloomWebApplicationFactory factory, string token, object? hello = null)
        {
            var client = factory.Server.CreateWebSocketClient();
            client.ConfigureRequest = req => req.Headers["Authorization"] = $"Bearer {token}";
            var ws = await client.ConnectAsync(new Uri("ws://localhost/ws/robot-agent"), CancellationToken.None);
            var socket = new TestShellSocket(ws);
            await socket.SendJsonAsync(hello ?? new { type = "hello", version = "1.0.7-g1a2b3c4d", commit = "1a2b3c4d5e6f" });
            return socket;
        }

        public static async Task<TestShellSocket> ConnectAdminAsync(
            BloomWebApplicationFactory factory, string? cookie, Guid robotId,
            bool force = false, string origin = SameOrigin, string? bearer = null)
        {
            var client = factory.Server.CreateWebSocketClient();
            client.ConfigureRequest = req =>
            {
                if (cookie != null)
                    req.Headers["Cookie"] = cookie;
                if (bearer != null)
                    req.Headers["Authorization"] = $"Bearer {bearer}";
                req.Headers["Origin"] = origin;
            };
            var ws = await client.ConnectAsync(
                new Uri($"ws://localhost/ws/shell?robotId={robotId}&cols=100&rows=30&force={force}"),
                CancellationToken.None);
            return new TestShellSocket(ws);
        }

        /// <summary>
        /// Logs in via POST /api/User/login and returns the Identity cookie as a Cookie header
        /// value. The WebSocket client can't share an HttpClient's cookie container.
        /// </summary>
        public static async Task<string> GetAuthCookieAsync(BloomWebApplicationFactory factory, Account account)
        {
            var client = factory.CreateClient(new WebApplicationFactoryClientOptions { HandleCookies = false });
            var response = await client.PostAsJsonAsync("api/User/login",
                new { Email = account.Email, Password = TestDataSeeder.DefaultPassword });
            response.EnsureSuccessStatusCode();

            var setCookie = response.Headers.GetValues("Set-Cookie")
                .First(c => c.StartsWith("bloom_cookie=", StringComparison.Ordinal));
            return setCookie.Split(';')[0];
        }

        /// <summary>Asserts the WebSocket handshake was refused with the given HTTP status.</summary>
        public static async Task<int> HandshakeStatusAsync(Func<Task> connect)
        {
            try
            {
                await connect();
            }
            catch (InvalidOperationException ex) when (ex.Message.Contains("status code"))
            {
                var digits = new string(ex.Message.Where(char.IsDigit).ToArray());
                return int.Parse(digits[^3..]);
            }
            return 101;
        }

        public static byte[] DataFrame(Guid sessionId, string text)
        {
            var payload = Encoding.UTF8.GetBytes(text);
            var frame = new byte[17 + payload.Length];
            frame[0] = 0x01;
            sessionId.TryWriteBytes(frame.AsSpan(1, 16), bigEndian: true, out _);
            payload.CopyTo(frame, 17);
            return frame;
        }
    }
}
