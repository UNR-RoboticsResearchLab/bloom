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
}