// bloom
// RobotSession.cs
// RobotSession model representing a sesssion with the robot. Tracks RobotState history and stores it in the database
// Created: 11/7/2025

namespace bloom.Models
{
    public class RobotStateHistory
    {
        public Guid Id { get; set; } = Guid.NewGuid().ToString();

        // Foreign key to the session
        public Guid RobotSessionId { get; set; }
        public RobotSession RobotSession { get; set; }

        // Snapshot of the robot state at that moment
        public RobotState RobotState { get; set; }

        public DateTime Timestamp { get; set; } = DateTime.UtcNow;
    }

}