// bloom
// LessonProgressDto.cs
// Data transfer objects for Lesson progress and interaction API operations
// Created: 02/21/2026

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for starting a lesson in a robot session
    /// </summary>
    public class StartLessonDto
    {
        public required Guid LessonId { get; set; }
    }

    /// <summary>
    /// Response DTO for lesson progress information
    /// </summary>
    public class LessonProgressResponseDto
    {
        public Guid Id { get; set; }
        public Guid LessonId { get; set; }
        public int CurrentStepId { get; set; }
        public string Status { get; set; } = "InProgress";
        public int TotalSteps { get; set; }
        public int CompletedSteps { get; set; }
        public DateTime StartedAt { get; set; }
        public DateTime? CompletedAt { get; set; }
    }

    /// <summary>
    /// Request DTO for updating lesson progress (sent by robot during lesson execution)
    /// </summary>
    public class UpdateLessonProgressDto
    {
        public int CurrentStepId { get; set; }
        public int CompletedSteps { get; set; }
        public string Status { get; set; } = "InProgress";  // "InProgress", "Completed", "Paused", "Failed"
    }

    /// <summary>
    /// Request DTO for logging student interactions during lessons
    /// </summary>
    public class LogLessonInteractionDto
    {
        public int StepId { get; set; }
        public string InteractionType { get; set; } = "Response";  // "Response", "RobotDialogue", "Note", "Question", "Timeout", "Fallback", "Feedback"
        public string? StudentResponse { get; set; }
        public bool? IsCorrect { get; set; }
        public int ResponseTimeMs { get; set; }
    }

    /// <summary>
    /// Response DTO for lesson interaction log
    /// </summary>
    public class LessonInteractionResponseDto
    {
        public Guid Id { get; set; }
        public int StepId { get; set; }
        public string InteractionType { get; set; } = string.Empty;
        public string? StudentResponse { get; set; }
        public bool? IsCorrect { get; set; }
        public DateTime Timestamp { get; set; }
        public int ResponseTimeMs { get; set; }
    }

    /// <summary>
    /// Request DTO for SLP to record feedback (approve/retry) on a lesson step.
    /// Used by: POST /api/lessoninteractions/{sessionId}/feedback
    /// </summary>
    public class RecordSLPFeedbackDto
    {
        /// <summary>
        /// The lesson step this feedback applies to.
        /// </summary>
        public int StepId { get; set; }

        /// <summary>
        /// The feedback command: "approve" (advance to next step) or "retry" (replay current step).
        /// Kept as string for forward-extensibility.
        /// </summary>
        public required string FeedbackCommand { get; set; }  // "approve" | "retry"
    }

    /// <summary>
    /// Response DTO for GET /api/robotsessions/{sessionId}/pending-feedback.
    /// Returned when an unacknowledged SLP feedback command exists for the session.
    /// </summary>
    public class PendingFeedbackResponseDto
    {
        public bool HasPendingFeedback { get; set; }
        public Guid? FeedbackId { get; set; }          // ID of the LessonInteraction row
        public int? StepId { get; set; }
        public string? FeedbackCommand { get; set; }   // "approve" | "retry"
        public DateTime? IssuedAt { get; set; }
    }

    /// <summary>
    /// Request DTO for the set-step step control command.
    /// </summary>
    public class SetStepDto
    {
        public required int TargetStep { get; set; }
    }

    /// <summary>
    /// Response DTO for a student's lesson progress record.
    /// </summary>
    public class StudentLessonProgressDto
    {
        public Guid Id { get; set; }
        public Guid LessonId { get; set; }
        public string LessonTitle { get; set; } = string.Empty;
        public int LessonStep { get; set; }
        public int ProgressPercentage { get; set; }
        public DateTime LastUpdated { get; set; }
    }

    /// <summary>
    /// In-memory step control command polled by the robot.
    /// Cleared once the robot acknowledges it.
    /// </summary>
    public class PendingStepControlDto
    {
        public string Command { get; set; } = string.Empty;  // "skip" | "replay" | "set_step"
        public int? TargetStep { get; set; }                 // Only set for "set_step"
    }
}
