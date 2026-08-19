// bloom
// RobotVoiceController.cs
// Browser-controlled live TTS voice override for a session, taking priority
// over the paired account's persistent RobotProfile.Voice. Polled by the
// robot the same way it polls for idle mode and pending lessons.

using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Lets a paired browser set a live, session-scoped TTS voice override, and
    /// lets the robot (and the browser) poll for the currently effective voice:
    /// session override, else the paired account's RobotProfile.Voice, else the
    /// system default.
    /// </summary>
    [Authorize(Policy = "JwtOrCookie")]
    [ApiController]
    [Route("api/robot-voice")]
    public class RobotVoiceController : BloomControllerBase
    {
        private readonly ILogger<RobotVoiceController> _logger;
        private readonly IRobotSessionService _sessionService;
        private readonly IVoiceOverrideService _voiceOverrideService;
        private readonly IRobotProfileService _robotProfileService;

        public RobotVoiceController(
            ILogger<RobotVoiceController> logger,
            IRobotSessionService sessionService,
            IVoiceOverrideService voiceOverrideService,
            IRobotProfileService robotProfileService)
        {
            _logger = logger;
            _sessionService = sessionService;
            _voiceOverrideService = voiceOverrideService;
            _robotProfileService = robotProfileService;
        }

        /// <summary>
        /// Sets a live voice override for a session. Applies immediately, including mid-lesson.
        /// </summary>
        [HttpPost("{sessionId}")]
        public async Task<IActionResult> SetVoice(Guid sessionId, [FromBody] SetVoiceDto dto)
        {
            try
            {
                if (!ModelState.IsValid)
                    return BadRequest(ModelState);

                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to set voice in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                _voiceOverrideService.SetOverride(sessionId, dto.Voice);
                _logger.LogInformation("Voice override set to {Voice} for session {SessionId}", dto.Voice, sessionId);
                return Ok(new { Message = "Voice updated", SessionId = sessionId, dto.Voice });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid voice requested for session {SessionId}", sessionId);
                return BadRequest(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error setting voice for session {SessionId}", sessionId);
                return BadRequest(new { message = $"Error setting voice: {ex.Message}" });
            }
        }

        /// <summary>
        /// Clears the session's voice override, reverting to the paired account's
        /// RobotProfile.Voice (or the system default). Always succeeds.
        /// </summary>
        [HttpPost("{sessionId}/reset")]
        public async Task<IActionResult> ResetVoice(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            _voiceOverrideService.ClearOverride(sessionId);
            _logger.LogInformation("Voice override reset for session {SessionId}", sessionId);
            return Ok(new { Message = "Voice reset", SessionId = sessionId });
        }

        /// <summary>
        /// Polling endpoint for the robot (and the browser panel) to read the effective
        /// voice for a session: override, else the paired account's profile voice, else default.
        /// </summary>
        [HttpGet("{sessionId}")]
        public async Task<IActionResult> GetVoice(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            var overrideVoice = _voiceOverrideService.GetOverride(sessionId);
            if (overrideVoice != null)
                return Ok(new VoiceResponseDto { Voice = overrideVoice });

            if (session.UserId != null)
            {
                var profile = await _robotProfileService.GetByAccountIdAsync(session.UserId);
                if (profile?.Voice != null)
                    return Ok(new VoiceResponseDto { Voice = profile.Voice });
            }

            return Ok(new VoiceResponseDto { Voice = RobotVoice.Default });
        }
    }
}
