namespace bloom.Models.dto
{
    public class LessonAiGenerateRequestDto
    {
        // Free-text description of the lesson to generate, e.g. "8yo, /r/ sounds, 5 steps".
        public required string Prompt { get; set; }

        // Optional in-progress lesson state from the builder form. When present, generation
        // revises within this context instead of starting from a blank lesson.
        public LessonDto? ExistingLesson { get; set; }
    }

    public class LessonAiGenerateStepRequestDto
    {
        // Full lesson context (title, objectives, all current steps) so the regenerated
        // step stays consistent with its neighbors.
        public required LessonDto Lesson { get; set; }

        // Index into Lesson.Steps identifying which step to generate or revise.
        public required int StepIndex { get; set; }

        // Optional free-text guidance, e.g. "make this shorter" or "add a question here".
        public string? Instructions { get; set; }
    }
}
