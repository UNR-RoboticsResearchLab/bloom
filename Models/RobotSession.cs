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

        // Timestamp when the session was created
        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;

        // Timestamp when the session was last updated
        public DateTime LastUpdatedAt { get; set; } = DateTime.UtcNow;

        // number of robots in the current session
        public int Robots { get; set; }

    }

}