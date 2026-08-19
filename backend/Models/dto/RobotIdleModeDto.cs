// bloom
// RobotIdleModeDto.cs
// Data transfer objects for browser-controlled robot idle mode (conversational vs. passive)
// when no lesson is active in a session.

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for setting a session's idle mode.
    /// </summary>
    public class SetIdleModeDto
    {
        /// <summary>
        /// "conversational" (free-form spoken conversation) or "passive" (quiet idle, mic off).
        /// </summary>
        public required string Mode { get; set; }
    }

    /// <summary>
    /// Response DTO for the current idle mode of a session.
    /// </summary>
    public class IdleModeResponseDto
    {
        public string Mode { get; set; } = "passive";
    }
}
