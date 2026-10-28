// bloom
// ShellSocket.cs
// Thin wrapper over a WebSocket for the remote shell relay. WebSocket allows one send and one
// receive at a time; the relay sends to a socket from other connections' loops (agent output goes
// out on the admin's socket), so sends are serialized here and bounded by a timeout so one stalled
// browser can't wedge a robot's receive loop.

using System.Buffers;
using System.Net.WebSockets;
using System.Text.Json;

namespace bloom.Services.RemoteShell
{
    public sealed class ShellSocket
    {
        private static readonly TimeSpan SendTimeout = TimeSpan.FromSeconds(10);

        private readonly WebSocket _ws;
        private readonly SemaphoreSlim _sendLock = new(1, 1);

        public ShellSocket(WebSocket ws)
        {
            _ws = ws;
        }

        public WebSocketState State => _ws.State;

        public Task SendJsonAsync<T>(T message) =>
            SendAsync(JsonSerializer.SerializeToUtf8Bytes(message, ShellProtocol.Json), WebSocketMessageType.Text);

        public Task SendBinaryAsync(ReadOnlyMemory<byte> data) =>
            SendAsync(data, WebSocketMessageType.Binary);

        private async Task SendAsync(ReadOnlyMemory<byte> data, WebSocketMessageType type)
        {
            using var timeout = new CancellationTokenSource(SendTimeout);
            try
            {
                await _sendLock.WaitAsync(timeout.Token);
            }
            catch (OperationCanceledException)
            {
                _ws.Abort();
                return;
            }

            try
            {
                if (_ws.State == WebSocketState.Open)
                    await _ws.SendAsync(data, type, endOfMessage: true, timeout.Token);
            }
            catch (Exception ex) when (ex is WebSocketException or OperationCanceledException)
            {
                // Peer is gone or stalled. Its own receive loop notices and cleans up.
                _ws.Abort();
            }
            finally
            {
                _sendLock.Release();
            }
        }

        /// <summary>
        /// Reads one complete message (reassembling fragments). Returns null when the peer closes,
        /// the socket fails, or the message exceeds ShellProtocol.MaxMessageBytes.
        /// </summary>
        public async Task<(WebSocketMessageType Type, byte[] Data)?> ReceiveAsync(CancellationToken ct)
        {
            var buffer = ArrayPool<byte>.Shared.Rent(16 * 1024);
            try
            {
                using var message = new MemoryStream();
                while (true)
                {
                    ValueWebSocketReceiveResult result;
                    try
                    {
                        result = await _ws.ReceiveAsync(buffer.AsMemory(), ct);
                    }
                    catch (Exception ex) when (ex is WebSocketException or OperationCanceledException)
                    {
                        return null;
                    }

                    if (result.MessageType == WebSocketMessageType.Close)
                        return null;

                    if (message.Length + result.Count > ShellProtocol.MaxMessageBytes)
                    {
                        await CloseAsync(WebSocketCloseStatus.MessageTooBig, "message too large");
                        return null;
                    }

                    message.Write(buffer, 0, result.Count);
                    if (result.EndOfMessage)
                        return (result.MessageType, message.ToArray());
                }
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(buffer);
            }
        }

        public async Task CloseAsync(WebSocketCloseStatus status, string description)
        {
            using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(5));
            try
            {
                await _sendLock.WaitAsync(timeout.Token);
            }
            catch (OperationCanceledException)
            {
                _ws.Abort();
                return;
            }

            try
            {
                if (_ws.State is WebSocketState.Open or WebSocketState.CloseReceived)
                    await _ws.CloseOutputAsync(status, description, timeout.Token);
            }
            catch (Exception ex) when (ex is WebSocketException or OperationCanceledException)
            {
                _ws.Abort();
            }
            finally
            {
                _sendLock.Release();
            }
        }
    }
}
