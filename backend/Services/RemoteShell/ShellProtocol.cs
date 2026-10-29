// bloom
// ShellProtocol.cs
// Vocabulary shared by the remote shell's two SignalR hubs. The wire format itself is SignalR's JSON
// hub protocol; the methods are the interfaces in ShellHubClients.cs (server -> client) and the
// public methods of RobotAgentHub / ShellHub (client -> server). Keep in sync with
// robot/agent/src/protocol.h and frontend/src/components/dashboard/RobotTerminal.jsx.

namespace bloom.Services.RemoteShell
{
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
}
