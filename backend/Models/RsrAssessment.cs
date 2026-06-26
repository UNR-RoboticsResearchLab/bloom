// bloom
// RsrAssessment.cs
// Stores the result of a Redmond Sentence Recall (RSR) assessment session.
// No user account required — each run is identified by a generated PID.

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class RsrAssessment
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        // Short human-readable participant ID generated at submission, e.g. "RSR-A3X9K2"
        public required string Pid { get; set; }

        public int AgeInMonths { get; set; }
        public int Percentile { get; set; }

        public int TotalScore { get; set; }

        // "Pass", "Fail", or "N/A"
        public required string Decision { get; set; }

        // Full JSON payload returned by the AutoRSR microservice
        public required string ResultJson { get; set; }

        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    }
}
