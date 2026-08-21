// bloom
// IIdleModeService.cs
// Interface for real-time robot idle mode control (conversational vs. passive),
// applicable only while no lesson is active in a session.

namespace bloom.Services
{
    public interface IIdleModeService
    {
        /// <summary>
        /// Sets the desired idle mode for a session. Throws ArgumentException for unrecognized modes.
        /// </summary>
        void SetIdleMode(Guid sessionId, string mode);

        /// <summary>
        /// Gets the current idle mode for a session. Defaults to "passive" if none has been set.
        /// </summary>
        string GetIdleMode(Guid sessionId);

        /// <summary>
        /// Clears any set idle mode for a session, reverting it to the default "passive" mode.
        /// </summary>
        void ClearIdleMode(Guid sessionId);
    }
}
