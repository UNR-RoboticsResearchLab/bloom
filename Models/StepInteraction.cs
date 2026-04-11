using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class StepInteraction
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        public bool WaitForResponse { get; set; }
        public int? MaxWaitSeconds { get; set; }

        public string? CorrectAnswer { get; set; }
        public string? CorrectResponseScript { get; set; }
        public string? IncorrectResponseScript { get; set; }

        public bool SingleTurnLlm { get; set; }
        public string? SingleTurnLlmPrompt { get; set; }

        public bool LlmFollowUp { get; set; }

        public string? FallbackScript { get; set; }

        /// <summary>
        /// JSON array of fallback visual aid filenames, e.g. ["fallback1.png", "fallback2.png"]
        /// </summary>
        public string? FallbackVisualAid { get; set; }

        /// <summary>
        /// JSON array of labels for fallback visual aids, e.g. ["Label 1", "Label 2"]
        /// </summary>
        public string? FallbackVisualAidLabels { get; set; }

        // Navigation
        public LessonStep LessonStep { get; set; } = null!;
    }
}
