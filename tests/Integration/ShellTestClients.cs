using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Http;
using System.Net.Http.Json;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Services.RemoteShell;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Http.Connections;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.AspNetCore.SignalR.Client;

namespace bloom.Tests.Integration
{
    internal sealed record AgentEvent(string Name, object?[] Args)
    {
        public T Arg<T>(int index) => (T)Args[index]!;
    }

    /// <summary>
    /// A robot agent as the C++ one behaves on the wire: SignalR over a WebSocket opened WITHOUT the
    /// negotiate step (SkipNegotiation), authenticated by an Authorization header. Everything the
    /// server sends is queued so a test can wait for it with a timeout instead of hanging.
    /// </summary>
    internal sealed class TestAgent : IAsyncDisposable
    {
        public static readonly TimeSpan DefaultTimeout = TimeSpan.FromSeconds(5);

        private readonly HubConnection _connection;
        private readonly Channel<AgentEvent> _events = Channel.CreateUnbounded<AgentEvent>();
        private readonly TaskCompletionSource _closed = new(TaskCreationOptions.RunContinuationsAsynchronously);

        private TestAgent(HubConnection connection)
        {
            _connection = connection;
            connection.On<Guid, int, int>("Open", (id, cols, rows) => Push("Open", id, cols, rows));
            connection.On<Guid, byte[]>("Input", (id, data) => Push("Input", id, data));
            connection.On<Guid, int, int>("Resize", (id, cols, rows) => Push("Resize", id, cols, rows));
            connection.On<Guid>("Close", id => Push("Close", id));
            connection.On<Guid, string>("SetSecret", (requestId, secret) => Push("SetSecret", requestId, secret));
            // Like the real agent: told why it's being dropped, it hangs up at once rather than
            // waiting for the server's fallback abort.
            connection.On<string>("Disconnecting", reason =>
            {
                Push("Disconnecting", reason);
                _ = connection.StopAsync();
            });
            connection.Closed += _ =>
            {
                _closed.TrySetResult();
                return Task.CompletedTask;
            };
        }

        private void Push(string name, params object?[] args) => _events.Writer.TryWrite(new AgentEvent(name, args));

        public Task Closed => _closed.Task;

        public static async Task<TestAgent> ConnectAsync(
            BloomWebApplicationFactory factory, string token, bool hello = true, string version = "1.0.7-g1a2b3c4d")
        {
            var connection = new HubConnectionBuilder()
                .WithUrl(new Uri("http://localhost/hubs/robot-agent"), options =>
                {
                    options.Transports = HttpTransportType.WebSockets;
                    options.SkipNegotiation = true;
                    options.HttpMessageHandlerFactory = _ => factory.Server.CreateHandler();
                    options.WebSocketFactory = async (context, ct) =>
                    {
                        var client = factory.Server.CreateWebSocketClient();
                        client.ConfigureRequest = req => req.Headers["Authorization"] = $"Bearer {token}";
                        return await client.ConnectAsync(context.Uri, ct);
                    };
                })
                .Build();

            var agent = new TestAgent(connection);
            await connection.StartAsync();
            if (hello)
                await connection.InvokeAsync("Hello", version, "1a2b3c4d5e6f");
            return agent;
        }

        public Task Hello(string version, string commit) => _connection.InvokeAsync("Hello", version, commit);
        public Task Output(Guid sessionId, string text) => _connection.InvokeAsync("Output", sessionId, Encoding.UTF8.GetBytes(text));
        public Task SessionExited(Guid sessionId, int? code = 0) => _connection.InvokeAsync("SessionExited", sessionId, code);
        public Task SecretApplied(Guid requestId, bool ok, string? message = null) =>
            _connection.InvokeAsync("SecretApplied", requestId, ok, message);

        public async Task<AgentEvent> NextAsync(TimeSpan? timeout = null)
        {
            using var cts = new CancellationTokenSource(timeout ?? DefaultTimeout);
            try
            {
                return await _events.Reader.ReadAsync(cts.Token);
            }
            catch (OperationCanceledException)
            {
                throw new TimeoutException("The server sent the agent nothing in time.");
            }
        }

        public async Task<AgentEvent> NextAsync(string name, TimeSpan? timeout = null)
        {
            var next = await NextAsync(timeout);
            if (next.Name != name)
                throw new InvalidOperationException($"Expected '{name}' but the server sent '{next.Name}'.");
            return next;
        }

        public async Task<bool> WaitClosedAsync(TimeSpan? timeout = null) =>
            await Task.WhenAny(Closed, Task.Delay(timeout ?? DefaultTimeout)) == Closed;

        public ValueTask DisposeAsync() => _connection.DisposeAsync();
    }

    /// <summary>
    /// An admin's browser terminal: SignalR with the normal negotiate step, the Identity cookie and an
    /// Origin header like a browser sends.
    /// </summary>
    internal sealed class TestAdmin : IAsyncDisposable
    {
        public const string SameOrigin = "http://localhost";

        private readonly HubConnection _connection;
        private readonly StringBuilder _output = new();
        private readonly object _lock = new();
        private readonly TaskCompletionSource<string> _sessionClosed = new(TaskCreationOptions.RunContinuationsAsynchronously);
        private readonly TaskCompletionSource _closed = new(TaskCreationOptions.RunContinuationsAsynchronously);

        private TestAdmin(HubConnection connection)
        {
            _connection = connection;
            connection.On<byte[]>("Output", data =>
            {
                lock (_lock) _output.Append(Encoding.UTF8.GetString(data));
            });
            // Like the browser: once the session is over there's nothing left on this connection.
            connection.On<string>("SessionClosed", reason =>
            {
                _sessionClosed.TrySetResult(reason);
                _ = connection.StopAsync();
            });
            connection.Closed += _ =>
            {
                _closed.TrySetResult();
                return Task.CompletedTask;
            };
        }

        public Task<string> SessionClosed => _sessionClosed.Task;
        public Task Closed => _closed.Task;

        public string Output
        {
            get { lock (_lock) return _output.ToString(); }
        }

        public static async Task<TestAdmin> ConnectAsync(
            BloomWebApplicationFactory factory, string? cookie, string origin = SameOrigin, string? bearer = null)
        {
            var connection = new HubConnectionBuilder()
                .WithUrl(new Uri("http://localhost/hubs/shell"), options =>
                {
                    options.HttpMessageHandlerFactory = _ => factory.Server.CreateHandler();
                    void Apply(IDictionary<string, string> headers)
                    {
                        if (cookie != null) headers["Cookie"] = cookie;
                        if (bearer != null) headers["Authorization"] = $"Bearer {bearer}";
                        headers["Origin"] = origin;
                    }
                    Apply(options.Headers);
                    options.WebSocketFactory = async (context, ct) =>
                    {
                        var client = factory.Server.CreateWebSocketClient();
                        client.ConfigureRequest = req =>
                        {
                            var headers = new Dictionary<string, string>();
                            Apply(headers);
                            foreach (var (key, value) in headers) req.Headers[key] = value;
                        };
                        return await client.ConnectAsync(context.Uri, ct);
                    };
                })
                .Build();

            var admin = new TestAdmin(connection);
            await connection.StartAsync();
            return admin;
        }

        public Task<OpenSessionResult> OpenSessionAsync(Guid robotId, bool force = false, int cols = 100, int rows = 30) =>
            _connection.InvokeAsync<OpenSessionResult>("OpenSession", robotId, cols, rows, force);

        public Task Input(string text) => _connection.InvokeAsync("Input", Encoding.UTF8.GetBytes(text));
        public Task Resize(int cols, int rows) => _connection.InvokeAsync("Resize", cols, rows);
        // Fire-and-forget, like the browser: the server answers by ending the session, which makes the
        // client hang up before a completion for this call could ever arrive.
        public Task CloseSession() => _connection.SendAsync("CloseSession");

        public async Task<string> WaitForOutputAsync(string contains, TimeSpan? timeout = null)
        {
            var deadline = DateTime.UtcNow + (timeout ?? TestAgent.DefaultTimeout);
            while (DateTime.UtcNow < deadline)
            {
                var text = Output;
                if (text.Contains(contains)) return text;
                await Task.Delay(20);
            }
            throw new TimeoutException($"Terminal output never contained '{contains}'. Got: '{Output}'");
        }

        public async Task<string> WaitSessionClosedAsync(TimeSpan? timeout = null)
        {
            var t = timeout ?? TestAgent.DefaultTimeout;
            if (await Task.WhenAny(SessionClosed, Task.Delay(t)) != SessionClosed)
                throw new TimeoutException("The session never closed.");
            return await SessionClosed;
        }

        public ValueTask DisposeAsync() => _connection.DisposeAsync();
    }

    internal static class ShellTestClients
    {
        /// <summary>
        /// Logs in via POST /api/User/login and returns the Identity cookie as a Cookie header
        /// value. The SignalR client doesn't share an HttpClient's cookie container.
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

        /// <summary>The HTTP status a refused SignalR connection attempt failed with.</summary>
        public static async Task<int> RefusedStatusAsync(Func<Task> connect)
        {
            try
            {
                await connect();
            }
            catch (HttpRequestException ex) when (ex.StatusCode is { } status)
            {
                return (int)status;  // refused at the negotiate request
            }
            catch (Exception ex) when (ex is InvalidOperationException or WebSocketException or AggregateException)
            {
                // Refused at the WebSocket upgrade (no negotiate step): TestServer reports the status in the text.
                var digits = new string(ex.Message.Where(char.IsDigit).ToArray());
                if (digits.Length >= 3) return int.Parse(digits[^3..]);
                throw;
            }
            return 200;
        }
    }
}
