// bloom
// LessonStep.cs
// Represents a single step in a lesson sequence.
// Behaviors and Interaction are stored as JSON strings since their schema varies by step type.

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class LessonStep
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        public Guid LessonId { get; set; }
        public Lesson Lesson { get; set; } = null!;

        public int StepOrder { get; set; }

        public required string Type { get; set; }

        public required string Script { get; set; }

        public int? TimingSeconds { get; set; }

        // May be a single filename or a JSON array of filenames
        public string? VisualAid { get; set; }

        // JSON string: { behavior, facial_expression, gaze, head_movement, posture, ... }
        public string? Behaviors { get; set; }

        // JSON string: schema varies by step type (wait_for_response, correct_answer, repeat_count, etc.)
        public string? Interaction { get; set; }
    }
}