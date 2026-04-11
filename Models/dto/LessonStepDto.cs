namespace bloom.Models.dto
{
    public class LessonStepDto
    {
        public Guid? Id { get; set; }
        public int StepOrder { get; set; }
        public required string Type { get; set; }
        public required string Script { get; set; }
        public int? TimingSeconds { get; set; }
        public string? VisualAid { get; set; }

        // JSON string — passed through as-is to/from the robot
        public string? Behaviors { get; set; }

        public StepInteractionDto? Interaction { get; set; }
    }
}