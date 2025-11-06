
namespace bloom.Models
{
    public class RobotState
    {
        public required string RobotId { get; set; }
        public required string Status { get; set; }
        public required string CurrentTask { get; set; }
        public DateTime LastUpdated { get; set; }
    }
}