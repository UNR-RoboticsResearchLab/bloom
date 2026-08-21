// bloom
// RobotVoiceDto.cs
// Data transfer objects for browser-controlled live TTS voice override,
// applied on top of the paired account's persistent RobotProfile.Voice.

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for setting a session's voice override.
    /// </summary>
    public class SetVoiceDto
    {
        public required string Voice { get; set; }
    }

    /// <summary>
    /// Response DTO for the effective voice of a session.
    /// </summary>
    public class VoiceResponseDto
    {
        public string Voice { get; set; } = bloom.Models.RobotVoice.Default;
    }
}
