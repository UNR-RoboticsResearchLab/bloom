// bloom
// IStepControlService.cs
// Interface for real-time step control commands (skip, replay, set_step).

using bloom.Models.dto;

namespace bloom.Services
{
    public interface IStepControlService
    {
        void SetPendingControl(Guid sessionId, string command, int? targetStep = null);
        PendingStepControlDto? GetPendingControl(Guid sessionId);
        void ClearControl(Guid sessionId);
    }
}
