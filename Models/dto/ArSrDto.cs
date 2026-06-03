namespace bloom.Models.dto
{
    public class ArSrEnrollRequest
    {
        public required string ParticipantCode { get; set; }
        public required string Condition { get; set; }
        public string? Gender { get; set; }
        public string? Ethnicity { get; set; }
        public string? NativeLanguage { get; set; }
        public bool? HearingLoss { get; set; }
        public bool? ReceivingSpeechTherapy { get; set; }
        public string? DemographicNotes { get; set; }
    }

    public class ArSrParticipantLoginRequest
    {
        public required string ParticipantCode { get; set; }
    }

    public class ArSrSessionRequest
    {
        public required Guid EnrollmentId { get; set; }
        public int AgeMonths { get; set; }
        public int Percentile { get; set; } = 5;
        public string? VideoFilePath { get; set; }
        public string? AdministratorNotes { get; set; }
    }

    public class ArSrEnrollmentDto
    {
        public Guid Id { get; set; }
        public string ParticipantAccountId { get; set; } = string.Empty;
        public string ParticipantCode { get; set; } = string.Empty;
        public string Condition { get; set; } = string.Empty;
        public DateTime EnrolledAt { get; set; }
        public string? Gender { get; set; }
        public string? Ethnicity { get; set; }
        public string? NativeLanguage { get; set; }
        public bool? HearingLoss { get; set; }
        public bool? ReceivingSpeechTherapy { get; set; }
        public int SessionCount { get; set; }
    }

    public class ArSrSessionDto
    {
        public Guid Id { get; set; }
        public Guid EnrollmentId { get; set; }
        public string ParticipantCode { get; set; } = string.Empty;
        public int AgeMonths { get; set; }
        public int Percentile { get; set; }
        public double? TotalScore { get; set; }
        public string? Result { get; set; }
        public string? VideoFilePath { get; set; }
        public DateTime CreatedAt { get; set; }
        public string? AdministratorNotes { get; set; }
        public List<ArSrSentenceResultDto> SentenceResults { get; set; } = new();
        public List<ArSrRecordingDto> Recordings { get; set; } = new();
    }

    public class ArSrSentenceResultDto
    {
        public int SentenceNumber { get; set; }
        public string GroundTruth { get; set; } = string.Empty;
        public string? Response { get; set; }
        public int Errors { get; set; }
        public int Score { get; set; }
        public string? EditScript { get; set; }
    }

    public class ArSrRecordingDto
    {
        public int SentenceNumber { get; set; }
        public string FilePath { get; set; } = string.Empty;
    }

    /// <summary>Shape returned by the Python microservice /api/analyze endpoint.</summary>
    public class ArSrMicroserviceResult
    {
        public string? SessionFolder { get; set; }
        public Dictionary<string, string> RecordingPaths { get; set; } = new();
        public double TotalScore { get; set; }
        public string? Result { get; set; }
        public List<ArSrMicroserviceSentence> Sentences { get; set; } = new();
    }

    public class ArSrMicroserviceSentence
    {
        public string Id { get; set; } = string.Empty;
        public string GroundTruth { get; set; } = string.Empty;
        public string? Response { get; set; }
        public int Errors { get; set; }
        public int Score { get; set; }
        public object? EditScript { get; set; }
    }
}
