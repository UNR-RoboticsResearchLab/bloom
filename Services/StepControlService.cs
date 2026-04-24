// bloom
// StepControlService.cs
// In-memory singleton for real-time step control commands (skip, replay, set_step).
// Commands are ephemeral — not persisted to the database.

using System.Collections.Concurrent;
using bloom.Models.dto;

namespace bloom.Services
{
    public class StepControlService : IStepControlService
    {
        private readonly ConcurrentDictionary<Guid, PendingStepControlDto> _pending = new();

        public void SetPendingControl(Guid sessionId, string command, int? targetStep = null)
        {
            _pending[sessionId] = new PendingStepControlDto
            {
                Command = command,
                TargetStep = targetStep
            };
        }

        public PendingStepControlDto? GetPendingControl(Guid sessionId)
        {
            _pending.TryGetValue(sessionId, out var control);
            return control;
        }

        public void ClearControl(Guid sessionId)
        {
            _pending.TryRemove(sessionId, out _);
        }
    }
}
