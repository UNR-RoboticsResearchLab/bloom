namespace bloom.Models.dto
{
    /// <summary>One robot as seen by the admin fleet view: identity, agent status, shell session.</summary>
    public class RobotFleetDto
    {
        public Guid Id { get; set; }
        public required string Name { get; set; }
        public string? Model { get; set; }
        public string? FirmwareVersion { get; set; }

        public bool Online { get; set; }
        public bool Provisioned { get; set; }
        public string? AgentVersion { get; set; }
        public string? AgentCommit { get; set; }
        public DateTime? LastSeenAt { get; set; }

        public string? ShellHeldBy { get; set; }
        public DateTime? ShellSince { get; set; }
    }
}
