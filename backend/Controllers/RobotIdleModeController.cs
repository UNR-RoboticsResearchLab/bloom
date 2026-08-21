// bloom
// RobotIdleModeController.cs
// Browser-controlled robot idle mode (conversational vs. passive), applicable
// only while no lesson is active in a session. Polled by the robot the same
// way it polls for pending lessons and step control.

using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Lets a paired browser set the robot's idle behavior (free-form conversation
    /// vs. plain passive waiting) while no lesson is running, and lets the robot
    /// poll for the current setting.
    /// </summary>
    [Authorize(Policy = "JwtOrCookie")]
    [ApiController]
    [Route("api/robot-idle-mode")]
    public class RobotIdleModeController : BloomControllerBase
    {
        private readonly ILogger<RobotIdleModeController> _logger;
        private readonly IRobotSessionService _sessionService;
        private readonly IIdleModeService _idleModeService;

        public RobotIdleModeController(
            ILogger<RobotIdleModeController> logger,
            IRobotSessionService sessionService,
            IIdleModeService idleModeService)
        {
            _logger = logger;
            _sessionService = sessionService;
            _idleModeService = idleModeService;
        }

        /// <summary>
        /// Sets the desired idle mode for a session. Rejected while a lesson is active.
        /// </summary>
        [HttpPost("{sessionId}")]
        public async Task<IActionResult> SetIdleMode(Guid sessionId, [FromBody] SetIdleModeDto dto)
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
                    _logger.LogWarning("User {UserId} attempted to set idle mode in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                if (session.ActiveLessonId != null)
                    return BadRequest(new { Message = "Idle mode cannot be changed while a lesson is active" });

                _idleModeService.SetIdleMode(sessionId, dto.Mode);
                _logger.LogInformation("Idle mode set to {Mode} for session {SessionId}", dto.Mode, sessionId);
                return Ok(new { Message = "Idle mode updated", SessionId = sessionId, dto.Mode });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid idle mode requested for session {SessionId}", sessionId);
                return BadRequest(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error setting idle mode for session {SessionId}", sessionId);
                return BadRequest(new { message = $"Error setting idle mode: {ex.Message}" });
            }
        }

        /// <summary>
        /// Stops any active idle mode, reverting the session to the default "passive" mode.
        /// Always succeeds — safe to call even if no idle mode is currently set.
        /// </summary>
        [HttpPost("{sessionId}/stop")]
        public async Task<IActionResult> StopIdleMode(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            _idleModeService.ClearIdleMode(sessionId);
            _logger.LogInformation("Idle mode stopped for session {SessionId}", sessionId);
            return Ok(new { Message = "Idle mode stopped", SessionId = sessionId });
        }

        /// <summary>
        /// Polling endpoint for the robot (and the browser panel) to read the current idle mode.
        /// </summary>
        [HttpGet("{sessionId}")]
        public async Task<IActionResult> GetIdleMode(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            return Ok(new IdleModeResponseDto { Mode = _idleModeService.GetIdleMode(sessionId) });
        }
    }
}
