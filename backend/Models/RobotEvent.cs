// bloom
// RobotEvent.cs
// Model representing an error, anomaly, timeout, etc. reported by a robot
// Created: 10/23/2025

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public enum RobotEventType
    {
        Error,
        Anomaly,
        Timeout,
        Warning,
        Info
    }

    public enum RobotEventSeverity
    {
        Low,
        Medium,
        High,
        Critical
    }

    public class RobotEvent
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        // Robot that reported the event
        public Guid RobotId { get; set; }
        public Robot? Robot { get; set; }

        // Session active when the event occurred (null if none, e.g. boot-time errors)
        public Guid? RobotSessionId { get; set; }
        public RobotSession? RobotSession { get; set; }

        public RobotEventType EventType { get; set; }
        public RobotEventSeverity Severity { get; set; } = RobotEventSeverity.Medium;

        // Robot-side component that raised the event (e.g. "rsr_speech_node", "lesson_poller")
        public string? Source { get; set; }

        // Short machine-readable code (e.g. "STT_TIMEOUT")
        public string? Code { get; set; }

        public required string Message { get; set; }

        // Optional free-form payload (stack trace, JSON context)
        public string? Details { get; set; }

        // When the robot observed the event (robot clock) vs. when the backend stored it
        public DateTime OccurredAt { get; set; } = DateTime.UtcNow;
        public DateTime ReceivedAt { get; set; } = DateTime.UtcNow;

        public bool IsResolved { get; set; }
        public DateTime? ResolvedAt { get; set; }
        public string? ResolutionNotes { get; set; }
    }
}
