// bloom
// RsrSpeechDto.cs
// Data transfer objects for queuing pre-generated RSR sentence audio to a robot's kiosk face.

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for queuing a sentence to be spoken by a robot.
    /// </summary>
    public class QueueRsrSpeechDto
    {
        public required int SentenceId { get; set; }
    }

    /// <summary>
    /// A queued sentence-play command, polled by the robot's kiosk face app.
    /// </summary>
    public class PendingRsrSpeechDto
    {
        public Guid CommandId { get; set; }
        public int SentenceId { get; set; }
        public string AudioUrl { get; set; } = string.Empty;
        public string VisemeUrl { get; set; } = string.Empty;
        public DateTime IssuedAt { get; set; }
    }

    /// <summary>
    /// One entry in the pre-generated RSR sentence manifest (backend/wwwroot/rsr-audio/manifest.json).
    /// </summary>
    public class RsrSentenceManifestEntryDto
    {
        public int Id { get; set; }
        public string Text { get; set; } = string.Empty;
        public string AudioUrl { get; set; } = string.Empty;
        public string VisemeUrl { get; set; } = string.Empty;
    }
}
