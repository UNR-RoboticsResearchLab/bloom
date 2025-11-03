
namespace bloom.Models
{
    public class RobotDto
    {
        public required string Name { get; set; }
        public required string Model { get; set; }
        public required string SerialNumber { get; set; }
        public DateTime ManufactureDate { get; set; }
        public required string FirmwareVersion { get; set; }
        public required string IPAddress { get; set; }
        public required string? RegisteredUserId { get; set; }
    }

    public class RobotConfigDto
    {
        public required string FirmwareVersion { get; set; }
        public required string IPAddress { get; set; }
    }

    public class RobotStateDto
    {
        public required string Status { get; set; }
        public DateTime LastActive { get; set; }
    }

    public class LessonInfoDto
    {
        public required string LessonId { get; set; }
        public required string Title { get; set; }
        public required string Description { get; set; }
    }

    public class SessionDto
    {
        public required string RobotId { get; set; }
        public required string UserId { get; set; }
        public DateTime StartTime { get; set; }
        public DateTime? EndTime { get; set; }
        public required string Status { get; set; }
    }
}