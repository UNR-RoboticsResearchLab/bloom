namespace bloom.Models
{
    /// <summary>
    /// Enrolls a Participant account into the Auto RSR study.
    /// Stores per-participant demographics and condition assignment.
    /// </summary>
    public class ArSrEnrollment
    {
        public Guid Id { get; set; } = Guid.NewGuid();

        /// <summary>FK to Account.Id of the Participant-role user.</summary>
        public string ParticipantAccountId { get; set; } = string.Empty;

        public string Condition { get; set; } = string.Empty;
        public DateTime EnrolledAt { get; set; } = DateTime.UtcNow;

        // Demographics
        public int? Age { get; set; }
        public string? Gender { get; set; }
        public string? Ethnicity { get; set; }
        public string? NativeLanguage { get; set; }
        public bool? HearingLoss { get; set; }
        public bool? ReceivingSpeechTherapy { get; set; }
        public string? DemographicNotes { get; set; }

        public ICollection<ArSrSession> Sessions { get; set; } = new List<ArSrSession>();
    }
}
