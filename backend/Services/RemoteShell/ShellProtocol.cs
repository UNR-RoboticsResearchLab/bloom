// bloom
// ShellProtocol.cs
// Wire format shared by both WebSocket legs of the remote shell. Keep in sync with
// robot/agent/src/protocol.h and frontend/src/components/dashboard/RobotTerminal.jsx.
//
// Control messages are JSON text frames: {"type": "...", ...}.
// Agent leg terminal data is binary: [1 byte frame type][16 byte session id, RFC 4122 order][payload].
// Admin leg terminal data is raw binary: one admin socket carries exactly one session.

using System.Text.Json;
using System.Text.Json.Serialization;

namespace bloom.Services.RemoteShell
{
    public static class ShellProtocol
    {
        public const byte DataFrame = 0x01;
        public const int HeaderLength = 17;

        // Generous for terminal traffic; bounds memory per connection.
        public const int MaxMessageBytes = 256 * 1024;

        public static readonly JsonSerializerOptions Json = new()
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        };

        public static byte[] EncodeDataFrame(Guid sessionId, ReadOnlySpan<byte> payload)
        {
            var frame = new byte[HeaderLength + payload.Length];
            frame[0] = DataFrame;
            sessionId.TryWriteBytes(frame.AsSpan(1, 16), bigEndian: true, out _);
            payload.CopyTo(frame.AsSpan(HeaderLength));
            return frame;
        }

        public static bool TryDecodeDataFrame(ReadOnlyMemory<byte> frame, out Guid sessionId, out ReadOnlyMemory<byte> payload)
        {
            sessionId = Guid.Empty;
            payload = default;
            if (frame.Length < HeaderLength || frame.Span[0] != DataFrame)
                return false;
            sessionId = new Guid(frame.Span.Slice(1, 16), bigEndian: true);
            payload = frame[HeaderLength..];
            return true;
        }
    }

    /// <summary>Why a shell session ended. Stored in ShellSessionAudit.EndReason.</summary>
    public static class ShellEndReason
    {
        public const string Exited = "exited";                       // shell process exited on the robot
        public const string AdminClosed = "admin_closed";
        public const string AdminDisconnected = "admin_disconnected";
        public const string TakenOver = "taken_over";                // another admin forced a new session
        public const string RobotDisconnected = "robot_disconnected";
        public const string Revoked = "revoked";
        public const string TokenExpired = "token_expired";
        public const string ServerShutdown = "server_shutdown";
        public const string CredentialsRotated = "credentials_rotated";
        public const string RobotDeleted = "robot_deleted";
    }

    /// <summary>Minimal envelope used to read the "type" field before dispatching.</summary>
    public record ShellMessage(string Type);

    // Agent -> backend
    public record AgentHello(string Type, string? Version, string? Commit);
    public record AgentExited(string Type, Guid SessionId, int? Code);
    public record AgentSecretApplied(string Type, Guid RequestId, bool Ok, string? Message);

    // Backend -> agent
    public record AgentOpen(Guid SessionId, int Cols, int Rows, string Type = "open");
    public record AgentResize(Guid SessionId, int Cols, int Rows, string Type = "resize");
    public record AgentClose(Guid SessionId, string Type = "close");
    // Carries a secret: never log this object.
    public record AgentSetSecret(Guid RequestId, string Secret, string Type = "set_secret");

    // Admin -> backend
    public record AdminResize(string Type, int Cols, int Rows);

    // Backend -> admin
    public record AdminOpened(Guid SessionId, string RobotName, string Type = "opened");
    public record AdminBusy(string? HeldBy, DateTime Since, string Type = "busy");
    public record AdminError(string Message, string Type = "error");
    public record AdminClosed(string Reason, string Type = "closed");
}
