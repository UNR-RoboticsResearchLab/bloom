namespace bloom.Models.dto
{
    public class StepInteractionDto
    {
        public Guid? Id { get; set; }

        public bool WaitForResponse { get; set; }
        public int? MaxWaitSeconds { get; set; }

        public string? CorrectAnswer { get; set; }
        public string? CorrectResponseScript { get; set; }
        public string? IncorrectResponseScript { get; set; }

        public bool SingleTurnLlm { get; set; }
        public string? SingleTurnLlmPrompt { get; set; }

        public bool LlmFollowUp { get; set; }

        public string? FallbackScript { get; set; }
        public string? FallbackVisualAid { get; set; }
        public string? FallbackVisualAidLabels { get; set; }
    }
}
