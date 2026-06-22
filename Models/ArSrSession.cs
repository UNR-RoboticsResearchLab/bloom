namespace bloom.Models
{
    /// <summary>
    /// One complete administration of the 16-item RSR screening for a participant.
    /// </summary>
    public class ArSrSession
    {
        public Guid Id { get; set; } = Guid.NewGuid();

        public Guid EnrollmentId { get; set; }
        public ArSrEnrollment? Enrollment { get; set; }

        /// <summary>Participant age in months at time of session (used for pass/fail thresholds).</summary>
        public int AgeMonths { get; set; }

        /// <summary>Percentile cutoff used for scoring (e.g. 5, 10, 15).</summary>
        public int Percentile { get; set; } = 5;

        /// <summary>Cumulative RSR score across all 16 items (each item is 0, 1, or 2).</summary>
        public double? TotalScore { get; set; }

        /// <summary>"Pass", "Fail", or "N/A".</summary>
        public string? Result { get; set; }

        /// <summary>Folder path containing all per-sentence audio recordings for this session.</summary>
        public string? SessionFolder { get; set; }

        /// <summary>Path or URL to the session video recording.</summary>
        public string? VideoFilePath { get; set; }

        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
        public string? AdministratorNotes { get; set; }

        public ICollection<ArSrSentenceResult> SentenceResults { get; set; } = new List<ArSrSentenceResult>();
        public ICollection<ArSrRecording> Recordings { get; set; } = new List<ArSrRecording>();
    }
}
