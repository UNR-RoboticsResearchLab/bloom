


namespace bloom.Models
{

    public class LessonInteraction
    {

        public Guid Id { get; set; } = Guid.NewGuid();
        public Guid? LessonId { get; set; }
        public Lesson? Lesson { get; set; }
        public Guid RobotSessionId { get; set; }
        public RobotSession? RobotSession { get; set; }



        public int StepId { get; set; }
        public string InteractionType { get; set; } = "Response";
        public string? StudentResponse { get; set; }
        public bool? IsCorrect { get; set; }
        public int ResponseTimeMs { get; set; }
        public DateTime Timestamp { get; set; } = DateTime.UtcNow;

        /// <summary>
        /// For SLPFeedback rows: "approve" or "retry". Null for all other interaction types.
        /// </summary>
        public string? FeedbackCommand { get; set; }

        /// <summary>
        /// For SLPFeedback rows: true once the robot has polled and acted on the command.
        /// </summary>
        public bool? IsAcknowledged { get; set; }

        /// <summary>
        /// Timestamp when the robot acknowledged the feedback. Null until acknowledged.
        /// </summary>
        public DateTime? AcknowledgedAt { get; set; }

    }

}