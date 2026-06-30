using Xunit;
using Moq;
using System;
using System.IO;
using System.Threading.Tasks;
using bloom.Services;
using Microsoft.AspNetCore.Hosting;
using Microsoft.Extensions.Logging;

namespace bloom.Tests.Services
{
    /// <summary>
    /// Unit tests for RsrSpeechService: manifest loading and the queue/pending/acknowledge
    /// lifecycle used to deliver pre-generated RSR sentence audio to a robot's kiosk face.
    /// </summary>
    public class RsrSpeechServiceTests : IDisposable
    {
        private readonly string _webRoot;
        private readonly RsrSpeechService _service;

        public RsrSpeechServiceTests()
        {
            _webRoot = Path.Combine(Path.GetTempPath(), "bloom-rsr-speech-tests-" + Guid.NewGuid());
            Directory.CreateDirectory(Path.Combine(_webRoot, "rsr-audio"));

            var env = new Mock<IWebHostEnvironment>();
            env.Setup(e => e.WebRootPath).Returns(_webRoot);

            var logger = new Mock<ILogger<RsrSpeechService>>();

            _service = new RsrSpeechService(env.Object, logger.Object);
        }

        private void WriteManifest(string json)
        {
            File.WriteAllText(Path.Combine(_webRoot, "rsr-audio", "manifest.json"), json);
        }

        public void Dispose()
        {
            if (Directory.Exists(_webRoot))
                Directory.Delete(_webRoot, recursive: true);
        }

        [Fact]
        public async Task GetSentenceManifestAsync_WhenManifestMissing_ReturnsEmptyList()
        {
            var manifest = await _service.GetSentenceManifestAsync();

            Assert.Empty(manifest);
        }

        [Fact]
        public async Task GetSentenceManifestAsync_WhenManifestPresent_ReturnsParsedEntries()
        {
            WriteManifest(@"[
                { ""id"": 1, ""text"": ""The big football player washed the car with the hose."",
                  ""audioUrl"": ""/rsr-audio/sentence_01.wav"", ""visemeUrl"": ""/rsr-audio/sentence_01.visemes.json"" }
            ]");

            var manifest = await _service.GetSentenceManifestAsync();

            var entry = Assert.Single(manifest);
            Assert.Equal(1, entry.Id);
            Assert.Equal("/rsr-audio/sentence_01.wav", entry.AudioUrl);
            Assert.Equal("/rsr-audio/sentence_01.visemes.json", entry.VisemeUrl);
        }

        [Fact]
        public async Task QueueSentenceAsync_WithUnknownSentenceId_Throws()
        {
            WriteManifest(@"[{ ""id"": 1, ""text"": ""hi"", ""audioUrl"": ""/a"", ""visemeUrl"": ""/v"" }]");
            var robotId = Guid.NewGuid();

            await Assert.ThrowsAsync<ArgumentException>(() => _service.QueueSentenceAsync(robotId, 99));
        }

        [Fact]
        public async Task QueueSentence_ThenPending_ThenAcknowledge_RoundTrips()
        {
            WriteManifest(@"[{ ""id"": 1, ""text"": ""hi"", ""audioUrl"": ""/rsr-audio/sentence_01.wav"", ""visemeUrl"": ""/rsr-audio/sentence_01.visemes.json"" }]");
            var robotId = Guid.NewGuid();

            Assert.Null(_service.GetPending(robotId));

            var commandId = await _service.QueueSentenceAsync(robotId, 1);

            var pending = _service.GetPending(robotId);
            Assert.NotNull(pending);
            Assert.Equal(commandId, pending!.CommandId);
            Assert.Equal(1, pending.SentenceId);
            Assert.Equal("/rsr-audio/sentence_01.wav", pending.AudioUrl);

            _service.ClearPending(robotId);

            Assert.Null(_service.GetPending(robotId));
        }

        [Fact]
        public async Task QueueSentence_ForDifferentRobots_DoesNotCrossDeliver()
        {
            WriteManifest(@"[
                { ""id"": 1, ""text"": ""one"", ""audioUrl"": ""/a1"", ""visemeUrl"": ""/v1"" },
                { ""id"": 2, ""text"": ""two"", ""audioUrl"": ""/a2"", ""visemeUrl"": ""/v2"" }
            ]");
            var robotA = Guid.NewGuid();
            var robotB = Guid.NewGuid();

            await _service.QueueSentenceAsync(robotA, 1);
            await _service.QueueSentenceAsync(robotB, 2);

            Assert.Equal(1, _service.GetPending(robotA)!.SentenceId);
            Assert.Equal(2, _service.GetPending(robotB)!.SentenceId);
        }
    }
}
