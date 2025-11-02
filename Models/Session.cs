// bloom
// Session.cs
// Model representing a Session entity
// Created: 11/1/2025

namespace bloom.Models
{
    public class Session
    {
        public required string Id { get; set; }
        public required string RobotId { get; set; }
        public required string UserId { get; set; }
        public required DateTime StartTime { get; set; }
        public DateTime? EndTime { get; set; }
        public required RobotState Status { get; set; }

        // Navigation properties
        public Account? User { get; set; }
        public Robot? Robot { get; set; }
    }
}