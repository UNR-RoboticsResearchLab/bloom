namespace bloom.Models.dto
{
    public class RobotDto
    {
        public required string Name { get; set; }
        public required string Model { get; set; }
        public required string SerialNumber { get; set; }
        public DateTime ManufactureDate { get; set; }
        public required string FirmwareVersion { get; set; }
        public required string IPAddress { get; set; }
        public string? RegisteredUserId { get; set; }
    }
}