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

        // 6-digit session code for easy session sharing/access
        public string? SessionCode { get; set; }

        // Timestamp when the session was created
        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;

        // Timestamp when the session was last updated
        public DateTime LastUpdatedAt { get; set; } = DateTime.UtcNow;

        // number of robots in the current session
        public int Robots { get; set; }

        // Active lesson currently being executed in this session (null if none)
        public Guid? ActiveLessonId { get; set; }
        public Lesson? ActiveLesson { get; set; }

        // The current run of ActiveLessonId. Mints a fresh value each time a lesson starts,
        // so repeating the same lesson in one session produces distinct interaction histories.
        public Guid? ActiveLessonRunId { get; set; }
        public LessonRun? ActiveLessonRun { get; set; }

        // Navigation property for historical state snapshots (for analysis features)
        public ICollection<RobotStateHistory>? StateHistory { get; set; }

    }

}