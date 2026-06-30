// bloom
// RsrSpeechService.cs
// In-memory singleton for queuing pre-generated RSR sentence audio commands to a robot's
// kiosk face app. Commands are ephemeral — not persisted to the database — mirroring
// StepControlService's pattern for real-time, poll-and-acknowledge robot commands.
// Sentence audio/viseme assets themselves are pre-generated offline by
// robot/scripts/generate_rsr_audio.py and served as static files from wwwroot/rsr-audio.

using System.Collections.Concurrent;
using System.Text.Json;
using bloom.Models.dto;

namespace bloom.Services
{
    public class RsrSpeechService : IRsrSpeechService
    {
        private readonly IWebHostEnvironment _env;
        private readonly ILogger<RsrSpeechService> _logger;
        private readonly ConcurrentDictionary<Guid, PendingRsrSpeechDto> _pending = new();

        private readonly SemaphoreSlim _manifestLock = new(1, 1);
        private IReadOnlyList<RsrSentenceManifestEntryDto>? _manifestCache;

        public RsrSpeechService(IWebHostEnvironment env, ILogger<RsrSpeechService> logger)
        {
            _env = env;
            _logger = logger;
        }

        public async Task<IReadOnlyList<RsrSentenceManifestEntryDto>> GetSentenceManifestAsync()
        {
            if (_manifestCache != null)
                return _manifestCache;

            await _manifestLock.WaitAsync();
            try
            {
                if (_manifestCache != null)
                    return _manifestCache;

                var manifestPath = Path.Combine(_env.WebRootPath ?? "wwwroot", "rsr-audio", "manifest.json");
                if (!File.Exists(manifestPath))
                {
                    _logger.LogWarning("RSR sentence manifest not found at {Path}. Run robot/scripts/generate_rsr_audio.py first.", manifestPath);
                    return Array.Empty<RsrSentenceManifestEntryDto>();
                }

                var json = await File.ReadAllTextAsync(manifestPath);
                var manifest = JsonSerializer.Deserialize<List<RsrSentenceManifestEntryDto>>(json, new JsonSerializerOptions
                {
                    PropertyNameCaseInsensitive = true
                }) ?? new List<RsrSentenceManifestEntryDto>();

                _manifestCache = manifest;
                return manifest;
            }
            finally
            {
                _manifestLock.Release();
            }
        }

        public async Task<Guid> QueueSentenceAsync(Guid robotId, int sentenceId)
        {
            var manifest = await GetSentenceManifestAsync();
            var sentence = manifest.FirstOrDefault(s => s.Id == sentenceId)
                ?? throw new ArgumentException($"Sentence {sentenceId} not found in manifest.", nameof(sentenceId));

            var commandId = Guid.NewGuid();
            _pending[robotId] = new PendingRsrSpeechDto
            {
                CommandId = commandId,
                SentenceId = sentence.Id,
                AudioUrl = sentence.AudioUrl,
                VisemeUrl = sentence.VisemeUrl,
                IssuedAt = DateTime.UtcNow
            };
            return commandId;
        }

        public PendingRsrSpeechDto? GetPending(Guid robotId)
        {
            _pending.TryGetValue(robotId, out var pending);
            return pending;
        }

        public void ClearPending(Guid robotId)
        {
            _pending.TryRemove(robotId, out _);
        }
    }
}
