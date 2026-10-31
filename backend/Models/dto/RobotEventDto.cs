namespace bloom.Models.dto
{
    /// <summary>Payload a robot sends to log an event.</summary>
    public class CreateRobotEventDto
    {
        public required Guid RobotId { get; set; }
        public Guid? RobotSessionId { get; set; }
        public required RobotEventType EventType { get; set; }
        public RobotEventSeverity Severity { get; set; } = RobotEventSeverity.Medium;
        public string? Source { get; set; }
        public string? Code { get; set; }
        public required string Message { get; set; }
        public string? Details { get; set; }

        // Robot-side timestamp; defaults to receipt time when omitted
        public DateTime? OccurredAt { get; set; }
    }

    /// <summary>Partial update (PUT); null fields are left unchanged.</summary>
    public class UpdateRobotEventDto
    {
        public RobotEventSeverity? Severity { get; set; }
        public bool? IsResolved { get; set; }
        public string? ResolutionNotes { get; set; }
    }

    /// <summary>Filters for event queries; all optional.</summary>
    public class RobotEventQueryDto
    {
        public Guid? RobotId { get; set; }
        public Guid? RobotSessionId { get; set; }
        public RobotEventType? EventType { get; set; }
        public RobotEventSeverity? MinSeverity { get; set; }
        public bool? IsResolved { get; set; }
        public DateTime? From { get; set; }
        public DateTime? To { get; set; }
        public int Skip { get; set; } = 0;
        public int Take { get; set; } = 100;
    }
}
