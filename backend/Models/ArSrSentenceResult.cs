namespace bloom.Models
{
    /// <summary>
    /// Scored result for one of the 16 RSR sentences within a session.
    /// Scoring: 2 pts = 0 errors, 1 pt = 1–3 errors, 0 pts = 4+ errors.
    /// </summary>
    public class ArSrSentenceResult
    {
        public Guid Id { get; set; } = Guid.NewGuid();

        public Guid SessionId { get; set; }
        public ArSrSession? Session { get; set; }

        /// <summary>Sentence number 1–16.</summary>
        public int SentenceNumber { get; set; }

        public string GroundTruth { get; set; } = string.Empty;

        /// <summary>Aligned transcription of what the participant said.</summary>
        public string? Response { get; set; }

        /// <summary>Total edit operations (insertions + deletions + substitutions + swaps).</summary>
        public int Errors { get; set; }

        /// <summary>Item score: 0, 1, or 2.</summary>
        public int Score { get; set; }

        /// <summary>JSON-serialized edit script: insertions, deletions, substitutions, swaps.</summary>
        public string? EditScript { get; set; }
    }
}
