// bloom
// RobotSession.cs
// RobotSession model representing a sesssion with the robot. Tracks RobotState history and stores it in the database
// Created: 11/7/2025

namespace bloom.Models

{
    // not necessarily going to be stored in the database, but we want to track the history of robot states in a session,
    // and then process them at the end of the session to generate reports and insights for the SLP and student
    public class RobotStateHistory
    {
        public Guid Id { get; set; } = Guid.NewGuid();

        // Foreign key to the session
        public Guid RobotSessionId { get; set; }
        public RobotSession? RobotSession { get; set; }

        // Snapshot of the robot state at that moment
        public required RobotState RobotState { get; set; }

        public DateTime Timestamp { get; set; } = DateTime.UtcNow;
    }

}