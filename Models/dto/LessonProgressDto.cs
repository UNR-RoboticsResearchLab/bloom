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
        public int TotalSteps { get; set; }
        public string Status { get; set; } = "InProgress";  // "InProgress", "Completed", "Paused", "Failed"
    }

    /// <summary>
    /// Request DTO for logging student interactions during lessons
    /// </summary>
    public class LogLessonInteractionDto
    {
        public int StepId { get; set; }
        public string InteractionType { get; set; } = "Response";  // "Response", "Question", "Timeout", "Fallback"
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
}
