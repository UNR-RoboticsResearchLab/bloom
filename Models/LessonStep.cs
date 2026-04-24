// bloom
// LessonStep.cs
// Represents a single step in a lesson sequence.
// Behaviors and Interaction are stored as JSON strings since their schema varies by step type.

using System.ComponentModel.DataAnnotations;
using System.Text.Json.Serialization;

namespace bloom.Models
{
    public class LessonStep
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        public Guid LessonId { get; set; }
        [JsonIgnore]
        public Lesson Lesson { get; set; } = null!;

        public int StepOrder { get; set; }

        public required string Type { get; set; }

        public required string Script { get; set; }

        public int? TimingSeconds { get; set; }

        // May be a single filename or a JSON array of filenames
        public string? VisualAid { get; set; }

        // JSON array of labels for visual aids, e.g. ["Sea", "See"]
        public string? VisualAidLabels { get; set; }

        // JSON array of footers for visual aids, e.g. ["1", "2"]
        public string? VisualAidFooters { get; set; }

        // Motor sequence to play for this step, e.g. "look_left"
        public string? MotorSequence { get; set; }

        // JSON string: { behavior, facial_expression, gaze, head_movement, posture, ... }
        public string? Behaviors { get; set; }

        // Structured interaction definition for this step (optional — not all steps have interactions)
        public Guid? InteractionId { get; set; }
        public StepInteraction? Interaction { get; set; }
    }
}