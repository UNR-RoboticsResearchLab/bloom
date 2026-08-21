// bloom
// VoiceOverrideService.cs
// In-memory singleton tracking each session's live TTS voice override, which
// takes priority over the paired account's persistent RobotProfile.Voice for
// the duration of the session. Ephemeral — not persisted to the database.

using System.Collections.Concurrent;
using bloom.Models;

namespace bloom.Services
{
    public class VoiceOverrideService : IVoiceOverrideService
    {
        private readonly ConcurrentDictionary<Guid, string> _overrides = new();

        public void SetOverride(Guid sessionId, string voiceId)
        {
            if (!RobotVoice.IsValid(voiceId))
                throw new ArgumentException($"'{voiceId}' is not a recognized voice.");

            _overrides[sessionId] = voiceId;
        }

        public string? GetOverride(Guid sessionId)
        {
            _overrides.TryGetValue(sessionId, out var voiceId);
            return voiceId;
        }

        public void ClearOverride(Guid sessionId)
        {
            _overrides.TryRemove(sessionId, out _);
        }
    }
}
