// bloom
// ShellSessionAudit.cs
// Audit record for one remote shell session: who opened a shell on which robot, when, and why it
// ended. Metadata only -- keystrokes and output are never stored.

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class ShellSessionAudit
    {
        [Key]
        public Guid Id { get; set; }

        public Guid RobotId { get; set; }

        public required string AdminUserId { get; set; }

        // Denormalized so the log stays readable after an account is deleted.
        public string? AdminEmail { get; set; }

        public DateTime StartedAt { get; set; }

        public DateTime? EndedAt { get; set; }

        // See ShellEndReason.
        public string? EndReason { get; set; }
    }
}
