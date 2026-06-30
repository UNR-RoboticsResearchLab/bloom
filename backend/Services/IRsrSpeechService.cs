// bloom
// IRsrSpeechService.cs
// Interface for queuing pre-generated RSR sentence audio for a robot's kiosk face to play.

using bloom.Models.dto;

namespace bloom.Services
{
    public interface IRsrSpeechService
    {
        Task<IReadOnlyList<RsrSentenceManifestEntryDto>> GetSentenceManifestAsync();

        /// <summary>
        /// Queues a sentence for a robot to play. Throws ArgumentException if sentenceId
        /// isn't present in the manifest.
        /// </summary>
        Task<Guid> QueueSentenceAsync(Guid robotId, int sentenceId);
        PendingRsrSpeechDto? GetPending(Guid robotId);
        void ClearPending(Guid robotId);
    }
}
