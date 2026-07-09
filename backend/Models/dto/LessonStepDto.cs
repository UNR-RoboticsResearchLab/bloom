using System.Text.Json.Serialization;

namespace bloom.Models.dto
{
    public class StepBehaviorsDto
    {
        [JsonPropertyName("behavior")]
        public string? Behavior { get; set; }

        [JsonPropertyName("facial_expression")]
        public string? FacialExpression { get; set; }

        [JsonPropertyName("gaze")]
        public string? Gaze { get; set; }

        [JsonPropertyName("head_movement")]
        public string? HeadMovement { get; set; }
    }

    public class LessonStepDto
    {
        public Guid? Id { get; set; }
        public int StepOrder { get; set; }
        public string? Title { get; set; }
        public required string Type { get; set; }
        public required string Script { get; set; }
        public int? TimingSeconds { get; set; }
        public string? VisualAid { get; set; }

        public string? VisualAidLabels { get; set; }
        public string? VisualAidFooters { get; set; }
        public string? MotorSequence { get; set; }
        public StepBehaviorsDto? Behaviors { get; set; }

        public StepInteractionDto? Interaction { get; set; }
    }
}
