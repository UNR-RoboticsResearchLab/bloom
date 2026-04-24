// bloom
// RobotSessionDto.cs
// Data transfer objects for RobotSession API operations
// Created: 11/18/2025

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for starting a new robot session
    /// </summary>
    public class StartSessionDto
    {
        /// <summary>
        /// Whether to create an anonymous session (null UserId)
        /// </summary>
        public bool Anonymous { get; set; } = false;
        /// <summary>
        /// RobotId to associate with the session
        /// </summary>
        public Guid RobotId { get; set; }
        /// <summary>
        /// Optional UserId for the session
        /// </summary>
        public string? UserId { get; set; }
    }

    /// <summary>
    /// Response DTO for session information
    /// </summary>
    public class RobotSessionResponseDto
    {
        public Guid Id { get; set; }
        public string? UserId { get; set; }
        public DateTime CreatedAt { get; set; }
        public DateTime LastUpdatedAt { get; set; }
        public int Robots { get; set; }
        public RobotState? LastState { get; set; }
        public Guid? ActiveLessonId { get; set; }
    }

    /// <summary>
    /// Request DTO for robot state input (used for both adding and updating states)
    /// </summary>
    public class RobotStateDto
    {
        public string? Status { get; set; } = "";
        public string? CurrentTask { get; set; } = "";
        public int? CurrentBehaviorId { get; set; }
        public string? SpeechLog { get; set; } = "";
    }

    /// <summary>
    /// Request DTO for adding a robot to a session
    /// </summary>
    public class AddRobotToSessionDto
    {
        public required Guid RobotId { get; set; }
        public required RobotStateDto CurrentState { get; set; }
    }

    /// <summary>
    /// Response DTO for robot state history
    /// </summary>
    public class RobotStateHistoryDto
    {
        public Guid Id { get; set; }
        public Guid RobotSessionId { get; set; }
        public Guid RobotId { get; set; }
        public string? Status { get; set; }
        public string? CurrentTask { get; set; }
        public DateTime Timestamp { get; set; }
    }


    /// <summary>
    /// DTO for tracker events related to robot sessions (for analytics and debugging)
    /// </summary>
    public class TrackerEventDto
    {
        public Guid Id { get; set; }
        public DateTime Timestamp { get; set; }

        public string Severity { get; set; } = "info";
        public string Source { get; set; } = string.Empty;
        public string EventType { get; set; } = string.Empty;
        public string Message { get; set; } = string.Empty;

        public Guid SessionId { get; set; }

        public string? LessonTitle { get; set; }
        public string? StudentName { get; set; }
        public string? RobotName { get; set; }

        public int? StepId { get; set; }
    }
}
