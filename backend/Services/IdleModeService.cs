// bloom
// IdleModeService.cs
// In-memory singleton tracking each session's current robot idle mode
// (conversational vs. passive). Unlike step control commands, idle mode is a
// persistent current value the robot re-applies idempotently on every poll,
// not a one-shot command that needs to be acknowledged/cleared by the robot.

using System.Collections.Concurrent;

namespace bloom.Services
{
    public class IdleModeService : IIdleModeService
    {
        public const string Conversational = "conversational";
        public const string Passive = "passive";

        private static readonly HashSet<string> ValidModes = new() { Conversational, Passive };

        private readonly ConcurrentDictionary<Guid, string> _idleMode = new();

        public void SetIdleMode(Guid sessionId, string mode)
        {
            if (!ValidModes.Contains(mode))
                throw new ArgumentException($"Unknown idle mode: {mode}");

            if (mode == Passive)
                _idleMode.TryRemove(sessionId, out _);
            else
                _idleMode[sessionId] = mode;
        }

        public string GetIdleMode(Guid sessionId) =>
            _idleMode.TryGetValue(sessionId, out var mode) ? mode : Passive;

        public void ClearIdleMode(Guid sessionId)
        {
            _idleMode.TryRemove(sessionId, out _);
        }
    }
}
