// bloom
// RsrDto.cs
// DTOs for the RSR assessment API.

namespace bloom.Models.dto
{
    public class RsrSentenceResultDto
    {
        public string Id { get; set; } = string.Empty;
        public string GroundTruth { get; set; } = string.Empty;
        public string Response { get; set; } = string.Empty;
        public int Errors { get; set; }
        public int Score { get; set; }
    }

    public class RsrAssessmentResultDto
    {
        public Guid AssessmentId { get; set; }
        public string Pid { get; set; } = string.Empty;
        public int TotalScore { get; set; }
        public string Decision { get; set; } = string.Empty;
        public List<RsrSentenceResultDto> Sentences { get; set; } = [];
        public DateTime CreatedAt { get; set; }
    }

    public class RsrAssessmentSummaryDto
    {
        public Guid Id { get; set; }
        public string Pid { get; set; } = string.Empty;
        public int AgeInMonths { get; set; }
        public int Percentile { get; set; }
        public int TotalScore { get; set; }
        public string Decision { get; set; } = string.Empty;
        public DateTime CreatedAt { get; set; }
    }
}
