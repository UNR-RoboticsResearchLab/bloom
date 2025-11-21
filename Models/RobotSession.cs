// bloom
// RobotSession.cs
// RobotSession model representing a sesssion with the robot. Tracks RobotState history and stores it in the database
// Created: 11/7/2025


namespace bloom.Models
{

    public class RobotSession
    {
        // Unique identifier for this session (e.g., tied to a user session)
        public Guid Id { get; set; } = Guid.NewGuid();

        // User who created/owns this session (null for anonymous sessions)
        public string? UserId { get; set; }
        public Account? User { get; set; }

        // Timestamp when the session was created
        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;

        // Timestamp when the session was last updated
        public DateTime LastUpdatedAt { get; set; } = DateTime.UtcNow;

        // number of robots in the current session
        public int Robots { get; set; }

        // Navigation property for historical state snapshots (for analysis features)
        public ICollection<RobotStateHistory>? StateHistory { get; set; }

    }

}