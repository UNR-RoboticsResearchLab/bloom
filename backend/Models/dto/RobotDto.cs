namespace bloom.Models.dto
{
    public class RobotDto
    {
        public required string Name { get; set; }
        public required string Model { get; set; }
        public string? FirmwareVersion { get; set; }
        public string? IPAddress { get; set; }
        public string? RegisteredUserId { get; set; }
    }

    /// <summary>
    /// Robot login. RobotId + Secret is the normal path. Name is only used for the
    /// Robot:AllowLegacyNameOnlyLogin rollout mode, for robots not yet provisioned with a secret.
    /// </summary>
    public class RobotLoginDto
    {
        public Guid? RobotId { get; set; }
        public string? Secret { get; set; }
        public string? Name { get; set; }
        public string? IPAddress { get; set; }
    }
}
