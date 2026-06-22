


namespace bloom.Models
{

    public class LessonInteraction
    {

        public Guid Id { get; set; } = Guid.NewGuid();
        public Guid? LessonId { get; set; }
        public Lesson? Lesson { get; set; }
        public Guid RobotSessionId { get; set; }
        public RobotSession? RobotSession { get; set; }

        /// <summary>
        /// Identifies the lesson run this interaction belongs to. Nullable so historical
        /// rows that predate the LessonRun concept remain queryable; new rows always set it.
        /// </summary>
        public Guid? LessonRunId { get; set; }
        public LessonRun? LessonRun { get; set; }



        public int StepId { get; set; }
        /// <summary>
        /// the type of the interaction
        ///   - robot: the robot response
        ///   - speaker: the speaker dialogue
        ///   - note: the note added by the SLP during the session
        /// </summary>
        public string InteractionType { get; set; } = string.Empty;
        public string? DialogTurn { get; set; }
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