// bloom
// RsrSpeechController.cs
// Delivers pre-generated RSR sentence audio to a robot's kiosk face app.
// No authentication required — RSR administration has no login, and robots are
// targeted by a pairing-code-resolved robotId, mirroring RsrController's public convention.

using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/rsr-speech")]
    public class RsrSpeechController : BloomControllerBase
    {
        private readonly IRsrSpeechService _rsrSpeech;
        private readonly IRobotService _robots;
        private readonly ILogger<RsrSpeechController> _logger;

        public RsrSpeechController(IRsrSpeechService rsrSpeech, IRobotService robots, ILogger<RsrSpeechController> logger)
        {
            _rsrSpeech = rsrSpeech;
            _robots = robots;
            _logger = logger;
        }

        /// <summary>
        /// Returns the pre-generated RSR sentence manifest (text, audio URL, viseme URL per sentence).
        /// </summary>
        [HttpGet("sentences")]
        public async Task<IActionResult> GetSentences()
        {
            var manifest = await _rsrSpeech.GetSentenceManifestAsync();
            return Ok(manifest);
        }

        /// <summary>
        /// Queues a sentence for the given robot's kiosk face to speak. Robot picks it up on next poll.
        /// </summary>
        [HttpPost("{robotId:guid}/queue")]
        public async Task<IActionResult> QueueSentence(Guid robotId, [FromBody] QueueRsrSpeechDto dto)
        {
            try
            {
                var robot = await _robots.GetRobotByIdAsync(robotId);
                if (robot == null)
                    return NotFound(new { Message = $"Robot with ID {robotId} not found" });

                var commandId = await _rsrSpeech.QueueSentenceAsync(robotId, dto.SentenceId);
                return Ok(new { Message = "Sentence queued for robot", RobotId = robotId, dto.SentenceId, CommandId = commandId });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid RSR sentence queue request for robot {RobotId}", robotId);
                return BadRequest(new { Message = ex.Message });
            }
        }

        /// <summary>
        /// Robot/kiosk polls for a queued sentence. Returns 200 with the command or 204 if none.
        /// </summary>
        [HttpGet("{robotId:guid}/pending")]
        public IActionResult GetPending(Guid robotId)
        {
            var pending = _rsrSpeech.GetPending(robotId);
            if (pending == null)
                return NoContent();

            return Ok(pending);
        }

        /// <summary>
        /// Robot/kiosk acknowledges a sentence after playback finishes, clearing it server-side.
        /// </summary>
        [HttpDelete("{robotId:guid}/pending")]
        public IActionResult AcknowledgePending(Guid robotId)
        {
            _rsrSpeech.ClearPending(robotId);
            return Ok(new { Message = "RSR speech command cleared", RobotId = robotId });
        }
    }
}
