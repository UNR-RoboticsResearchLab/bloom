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

        public LessonSessionController(
            ILogger<LessonSessionController> logger,
            IRobotSessionService sessionService)
        {
            _logger = logger;
            _sessionService = sessionService;
        }

        /// <summary>
        /// Start a lesson in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Lesson start data</param>
        /// <returns>Success message with lesson details</returns>
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
                if (session.UserId != null && session.UserId != currentUserId)
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

        private string? GetCurrentUserId()
        {
            return User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
        }
    }
}
