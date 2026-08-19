// bloom
// IVoiceOverrideService.cs
// Interface for a live, per-session TTS voice override that takes priority
// over the paired account's RobotProfile.Voice for the duration of a session.

namespace bloom.Services
{
    public interface IVoiceOverrideService
    {
        /// <summary>
        /// Sets the voice override for a session. Throws ArgumentException for unrecognized voices.
        /// </summary>
        void SetOverride(Guid sessionId, string voiceId);

        /// <summary>
        /// Gets the voice override for a session, or null if none is set.
        /// </summary>
        string? GetOverride(Guid sessionId);

        /// <summary>
        /// Clears the voice override for a session.
        /// </summary>
        void ClearOverride(Guid sessionId);
    }
}
