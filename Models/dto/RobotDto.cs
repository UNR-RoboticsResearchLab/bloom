
namespace bloom.Models
{
    public class RobotDto
    {
        public string Name { get; set; }
        public string Model { get; set; }
        public string SerialNumber { get; set; }
        public DateTime ManufactureDate { get; set; }
        public string FirmwareVersion { get; set; }
        public string IPAddress { get; set; }
        public string? RegisteredUserId { get; set; }
    }
}