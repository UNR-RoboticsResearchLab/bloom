// bloom
// ShellHubClients.cs
// What the relay can call on the other end of each hub (SignalR "strongly typed hubs"), and what
// ShellHub.OpenSession answers. Byte payloads travel as base64 in SignalR's JSON protocol.

namespace bloom.Services.RemoteShell
{
    /// <summary>Server -> robot agent. Targets are these method names.</summary>
    public interface IAgentClient
    {
        Task Open(Guid sessionId, int cols, int rows);
        Task Input(Guid sessionId, byte[] data);
        Task Resize(Guid sessionId, int cols, int rows);
        Task Close(Guid sessionId);

        // Carries a secret: never log arguments.
        Task SetSecret(Guid requestId, string secret);

        // Sent just before the server drops the connection, because a hub can't attach a reason to
        // an abort. The agent uses it to decide how to reconnect (see ShellEndReason).
        Task Disconnecting(string reason);
    }

    /// <summary>Server -> admin's browser terminal.</summary>
    public interface IAdminClient
    {
        Task Output(byte[] data);
        Task SessionClosed(string reason);
    }

    /// <summary>Answer to ShellHub.OpenSession. Expected refusals are statuses, not exceptions.</summary>
    public record OpenSessionResult(
        string Status,            // opened | offline | busy | rate_limited | not_found
        Guid? SessionId = null,
        string? RobotName = null,
        string? HeldBy = null,
        DateTime? Since = null,
        string? Message = null);
}
