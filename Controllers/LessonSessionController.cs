// bloom
// LessonSessionController.cs
// API controller for managing lesson lifecycle within an active robot session

using System.Security.Claims;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages the lesson lifecycle within an active robot session.
    /// Handles starting a lesson, polling for a pending lesson assignment,
    /// and updating lesson progress as the robot advances through steps.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class LessonSessionController : ControllerBase
    {
        private readonly ILogger<LessonSessionController> _logger;
        private readonly IRobotSessionService _sessionService;
        private readonly IStepControlService _stepControlService;

        public LessonSessionController(
            ILogger<LessonSessionController> logger,
            IRobotSessionService sessionService,
            IStepControlService stepControlService)
        {
            _logger = logger;
            _sessionService = sessionService;
            _stepControlService = stepControlService;
        }

        /// <summary>
        /// Start a lesson in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Lesson start data</param>
        /// <returns>Success message with lesson details</returns>
        /// <response code="200">Lesson started successfully</response>
        /// <response code="400">Invalid request data</response>
        /// <response code="403">User not authorized to start lesson in this session</response>
        /// <response code="404">Session not found</response>
        [HttpPost("{sessionId}/lesson")]
        public async Task<IActionResult> StartLesson(Guid sessionId, [FromBody] StartLessonDto dto)
        {
            try
            {
                if (!ModelState.IsValid)
                {
                    return BadRequest(ModelState);
                }

                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to start lesson in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                var lessonSession = await _sessionService.StartLessonAsync(sessionId, dto);

                return Ok(new {
                    Message = "Lesson started successfully",
                    SessionId = sessionId,
                    LessonId = lessonSession.ActiveLessonId,
                    LessonTitle = lessonSession.ActiveLesson?.Title,
                    TotalSteps = lessonSession.ActiveLesson?.TotalSteps
                });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
        }

        /// <summary>
        /// Polling endpoint for the robot to check if a lesson has been queued and auto-start it.
        /// Returns 200 with lesson details if a lesson is pending, or 204 if nothing is queued.
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        [HttpGet("{sessionId}/pending-lesson")]
        public async Task<IActionResult> GetPendingLesson(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var pendingLesson = await _sessionService.GetPendingLessonAsync(sessionId);

                if (pendingLesson == null)
                {
                    return NoContent();
                }

                return Ok(pendingLesson);
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving pending lesson for session {SessionId}", sessionId);
                return BadRequest(new { message = $"Error retrieving pending lesson: {ex.Message}" });
            }
        }

        /// <summary>
        /// Update lesson progress for a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Lesson progress update data</param>
        /// <returns>Success message with updated progress details</returns>
        [HttpPut("{sessionId}/lessons/progress")]
        public async Task<IActionResult> UpdateLessonProgress(Guid sessionId, [FromBody] UpdateLessonProgressDto dto)
        {
            try
            {
                if (!ModelState.IsValid)
                {
                    return BadRequest(ModelState);
                }

                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var currentUserId = GetCurrentUserId();
                if (currentUserId != null && session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to update lesson progress in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                await _sessionService.UpdateLessonProgressAsync(sessionId, dto);

                return Ok(new
                {
                    Message = "Lesson progress updated successfully",
                    SessionId = sessionId,
                    CurrentStepId = dto.CurrentStepId,
                    CompletedSteps = dto.CompletedSteps,
                    Status = dto.Status
                });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid argument when updating lesson progress");
                return BadRequest(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error updating lesson progress for session {SessionId}", sessionId);
                return StatusCode(500, "Internal server error");
            }
        }


        /// <summary>
        /// Skip the current lesson step. The robot will advance to the next step on its next poll.
        /// </summary>
        [HttpPost("{sessionId}/lessons/skip")]
        public async Task<IActionResult> SkipStep(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId == null)
                return BadRequest(new { Message = "No active lesson to skip" });

            _stepControlService.SetPendingControl(sessionId, "skip");
            _logger.LogInformation("Skip command issued for session {SessionId}", sessionId);
            return Ok(new { Message = "Skip command queued", SessionId = sessionId });
        }

        /// <summary>
        /// Replay the current lesson step. The robot will restart the current step on its next poll.
        /// </summary>
        [HttpPost("{sessionId}/lessons/replay")]
        public async Task<IActionResult> ReplayStep(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId == null)
                return BadRequest(new { Message = "No active lesson to replay" });

            _stepControlService.SetPendingControl(sessionId, "replay");
            _logger.LogInformation("Replay command issued for session {SessionId}", sessionId);
            return Ok(new { Message = "Replay command queued", SessionId = sessionId });
        }

        /// <summary>
        /// Jump to a specific lesson step. The robot will move to the given step on its next poll.
        /// </summary>
        [HttpPost("{sessionId}/lessons/set-step")]
        public async Task<IActionResult> SetStep(Guid sessionId, [FromBody] SetStepDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId == null)
                return BadRequest(new { Message = "No active lesson to set step on" });

            _stepControlService.SetPendingControl(sessionId, "set_step", dto.TargetStep);
            _logger.LogInformation("Set step {TargetStep} command issued for session {SessionId}", dto.TargetStep, sessionId);
            return Ok(new { Message = "Set step command queued", SessionId = sessionId, dto.TargetStep });
        }

        /// <summary>
        /// Polling endpoint for the robot to check for a pending step control command.
        /// Returns 200 with command details if one is queued, or 204 if none.
        /// </summary>
        [HttpGet("{sessionId}/lessons/step-control")]
        public async Task<IActionResult> GetPendingStepControl(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            var control = _stepControlService.GetPendingControl(sessionId);
            if (control == null)
                return NoContent();

            return Ok(control);
        }

        /// <summary>
        /// Acknowledge and clear the pending step control command for a session.
        /// Called by the robot after it has acted on the command.
        /// </summary>
        [HttpDelete("{sessionId}/lessons/step-control")]
        public async Task<IActionResult> AcknowledgeStepControl(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            _stepControlService.ClearControl(sessionId);
            _logger.LogInformation("Step control acknowledged and cleared for session {SessionId}", sessionId);
            return Ok(new { Message = "Step control cleared", SessionId = sessionId });
        }

        /// <summary>
        /// Get the lesson run history for a given student. Each entry is a single lesson run
        /// of a lesson within a robot session; repeating the same lesson yields multiple entries.
        /// </summary>
        /// <param name="studentId">Account ID of the student</param>
        [HttpGet("student/{studentId}/history")]
        public async Task<IActionResult> GetStudentLessonHistory(string studentId)
        {
            if (string.IsNullOrWhiteSpace(studentId))
                return BadRequest(new { Message = "studentId is required" });

            var history = await _sessionService.GetStudentLessonHistoryAsync(studentId);
            return Ok(history);
        }


        /// <summary>
        /// Stop the current lesson in a session. The robot will exit the lesson and return to idle state.
        /// </summary>
        [HttpPost("{sessionId}/lessons/stop")]
        public async Task<IActionResult> StopLesson(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId != null)
            {
                _stepControlService.SetPendingControl(sessionId, "stop");
            }

            await _sessionService.StopLessonAsync(sessionId);
            _logger.LogInformation("Stop command issued for session {SessionId}", sessionId);
            return Ok(new { Message = "Stop command queued", SessionId = sessionId });
        }

        private string? GetCurrentUserId()
        {
            return User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
        }
    }
}
