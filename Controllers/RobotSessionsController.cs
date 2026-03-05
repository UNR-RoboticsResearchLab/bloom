// bloom
// RobotSessionController.cs
// API controller for managing robot sessions and real-time state monitoring
// Created: 11/18/2025

using System.Security.Claims;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore.Update.Internal;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class RobotSessionsController : ControllerBase
    {
        private readonly ILogger<RobotSessionsController> _logger;
        private readonly IRobotSessionService _sessionService;
        private readonly IRobotService _robotService;
        private readonly IAccountService _accountService;

        public RobotSessionsController(
            ILogger<RobotSessionsController> logger,
            IRobotSessionService sessionService,
            IRobotService robotService,
            IAccountService accountService)
        {
            _logger = logger;
            _sessionService = sessionService;
            _robotService = robotService;
            _accountService = accountService;
        }

        /// <summary>
        /// Gets the current authenticated user's ID from claims
        /// </summary>
        private string? GetCurrentUserId()
        {
            return User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
        }

        /// <summary>
        /// Get all robot sessions ordered by creation date (newest first)
        /// </summary>
        /// <returns>Collection of all RobotSessions</returns>
        [HttpGet]
        public async Task<IActionResult> GetSessions()
        {
            try
            {
                var sessions = await _sessionService.GetAllSessionsAsync();
                var sessionDtos = new List<RobotSessionResponseDto>();

                foreach (var session in sessions)
                {
                    var robotIds = (await _sessionService.GetSessionRobotsAsync(session.Id)).ToList();
                    sessionDtos.Add(new RobotSessionResponseDto
                    {
                        Id = session.Id,
                        UserId = session.UserId,
                        CreatedAt = session.CreatedAt,
                        LastUpdatedAt = session.LastUpdatedAt,
                        Robots = session.Robots,
                        RobotIds = robotIds
                    });
                }

                return Ok(sessionDtos);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving all robot sessions");
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Start a new robot session
        /// </summary>
        /// <param name="dto">Session configuration</param>
        /// <returns>The created RobotSession</returns>
        [HttpPost]
        public async Task<IActionResult> StartSession([FromBody] StartSessionDto dto)
        {
            try
            {
                if (!ModelState.IsValid)
                {
                    return BadRequest(ModelState);
                }

                var userId = GetCurrentUserId();

                // For anonymous sessions, userId can be null
                // For authenticated sessions, verify user exists
                if (!dto.Anonymous && userId != null)
                {
                    var user = await _accountService.GetByIdAsync(userId);
                    if (user == null)
                    {
                        _logger.LogWarning("Attempted to start session with non-existent user {UserId}", userId);
                        return Unauthorized();
                    }
                }

                var session = await _sessionService.StartSessionAsync(dto.RobotId, userId, dto.Anonymous);
                var robotIds = (await _sessionService.GetSessionRobotsAsync(session.Id)).ToList();

                return CreatedAtAction(nameof(GetSession), new { sessionId = session.Id }, new RobotSessionResponseDto
                {
                    Id = session.Id,
                    UserId = session.UserId,
                    CreatedAt = session.CreatedAt,
                    LastUpdatedAt = session.LastUpdatedAt,
                    Robots = session.Robots,
                    RobotIds = robotIds
                });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error starting robot session");
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Get session details
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>RobotSession details</returns>
        [HttpGet("{sessionId}")]
        public async Task<IActionResult> GetSession(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var robotIds = (await _sessionService.GetSessionRobotsAsync(sessionId)).ToList();

                return Ok(new RobotSessionResponseDto
                {
                    Id = session.Id,
                    UserId = session.UserId,
                    CreatedAt = session.CreatedAt,
                    LastUpdatedAt = session.LastUpdatedAt,
                    Robots = session.Robots,
                    RobotIds = robotIds
                });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robot session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// End a robot session and archive remaining states
        /// </summary>
        /// <param name="sessionId">ID of the session to end</param>
        /// <returns>Success message</returns>
        [HttpPost("{sessionId}/end")]
        public async Task<IActionResult> EndSession(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                // Verify ownership if authenticated
                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to end session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                await _sessionService.EndSessionAsync(sessionId);

                return Ok(new { Message = "Session ended successfully", SessionId = sessionId });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error ending robot session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Polling endpoint for robot to see if a pending lesson exists and auto-start it
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Pending lesson details if exists, otherwise 204 No Content</returns>
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
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Add a robot to an active session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Robot state to add</param>
        /// <returns>Success message</returns>
        [HttpPost("{sessionId}/robots")]
        public async Task<IActionResult> AddRobot(Guid sessionId, [FromBody] StartSessionDto dto)
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

                // Verify session ownership if authenticated
                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to add robot to session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }


                await _sessionService.AddRobotToSessionAsync(sessionId, dto.RobotId);

                return Ok(new { Message = "Robot added to session successfully", SessionId = sessionId, RobotId = dto.RobotId });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid argument when adding robot to session");
                return BadRequest(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error adding robot to session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Remove a robot from a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="robotId">ID of the robot to remove</param>
        /// <returns>Success message</returns>
        [HttpDelete("{sessionId}/robots/{robotId}")]
        public async Task<IActionResult> RemoveRobot(Guid sessionId, Guid robotId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                // Verify session ownership if authenticated
                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to remove robot from session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                await _sessionService.RemoveRobotFromSessionAsync(sessionId, robotId);

                return Ok(new { Message = "Robot removed from session successfully", SessionId = sessionId, RobotId = robotId });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error removing robot from session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Get all robots currently in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of robot IDs</returns>
        [HttpGet("{sessionId}/robots")]
        public async Task<IActionResult> GetSessionRobots(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var robots = await _sessionService.GetSessionRobotsAsync(sessionId);

                return Ok(new { SessionId = sessionId, RobotIds = robots });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots for session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Update lesson progress for a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Lesson progress update data</param>
        /// <returns>Success message with updated session ID</returns>
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

                // Verify session ownership if authenticated
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

        /// <summary>
        /// Log a student interaction during a lesson
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Interaction log data</param>
        /// <returns>Success message with interaction ID</returns>
        [HttpPost("{sessionId}/lessons/interactions")]
        public async Task<IActionResult> LogLessonInteraction(Guid sessionId, [FromBody] LogLessonInteractionDto dto)
        {
            if (!ModelState.IsValid)
            {
                return BadRequest(ModelState);
            }

            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                // Verify session ownership if authenticated
                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to log interaction in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                var interactionId = await _sessionService.LogLessonInteractionAsync(sessionId, dto);

                return Ok(new
                {
                    Message = "Lesson interaction logged successfully",
                    SessionId = sessionId,
                    InteractionId = interactionId
                });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid argument when logging interaction");
                return BadRequest(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error logging lesson interaction for session {SessionId}", sessionId);
                return StatusCode(500, "Internal server error");
            }
        }

        /// <summary>
        /// Update a robot's state in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="robotId">ID of the robot</param>
        /// <param name="dto">New robot state</param>
        /// <returns>Success message</returns>
        [HttpPut("{sessionId}/robots/{robotId}/state")]
        public async Task<IActionResult> UpdateRobotState(
            Guid sessionId,
            Guid robotId,
            [FromBody] RobotStateDto dto)
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

                // Verify session ownership if authenticated
                var currentUserId = GetCurrentUserId();
                if (session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to update robot state in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                // Create new state from DTO
                var newState = new RobotState
                {
                    Id = Guid.NewGuid(),
                    RobotId = robotId,
                    Status = dto.Status ?? "",
                    CurrentTask = dto.CurrentTask ?? "",
                    CurrentBehaviorId = dto.CurrentBehaviorId,
                    LastStatusChange = DateTime.UtcNow,
                    SpeechLog = dto.SpeechLog ?? ""
                };

                await _sessionService.UpdateRobotStateAsync(sessionId, robotId, newState);

                return Ok(new { Message = "Robot state updated successfully", SessionId = sessionId, RobotId = robotId });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid argument when updating robot state");
                return BadRequest(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error updating robot state in session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }


        /// <summary>
        /// Sets the approve or disapprove tag on a session with an active lesson
        /// </summary>
        /// <param name="sessionId"> ID of sesion</param>
        /// <param name="feedback"> "approve" or "disapprove" feedback from SLP. true, false respectively</param>
        /// <returns>success/failure of the request</returns>
        [HttpPost("{sessionId}/lesson")]
        public async Task<IActionResult> ApproveOrDisapproveLessonStage(string sessionId, [FromBody] bool feedback)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(new Guid(sessionId));

                

                return Ok();
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robot states for session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Get all currently active robot states in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of active RobotStates</returns>
        [HttpGet("{sessionId}/states")]
        public async Task<IActionResult> GetCurrentStates(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var states = await _sessionService.GetCurrentStatesAsync(sessionId);

                return Ok(new { SessionId = sessionId, States = states });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robot states for session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Get historical state snapshots for a session (for analysis)
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of historical state snapshots</returns>
        [HttpGet("{sessionId}/history")]
        public async Task<IActionResult> GetSessionHistory(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);

                if (session == null)
                {
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });
                }

                var history = await _sessionService.GetSessionHistoryAsync(sessionId);

                var historyDtos = history.Select(h => new RobotStateHistoryDto
                {
                    Id = h.Id,
                    RobotSessionId = h.RobotSessionId,
                    RobotId = h.RobotState.RobotId,
                    Status = h.RobotState.Status,
                    CurrentTask = h.RobotState.CurrentTask,
                    Timestamp = h.Timestamp
                }).ToList();

                return Ok(new { SessionId = sessionId, History = historyDtos });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving session history for {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Robot polls this endpoint to check if an SLP has issued a feedback command
        /// (approve or retry) that has not yet been acknowledged.
        /// Returns 200 with command data, or 204 if no pending feedback.
        /// GET /api/robotsessions/{sessionId}/pending-feedback
        /// </summary>
        [HttpGet("{sessionId}/pending-feedback")]
        public async Task<IActionResult> GetPendingFeedback(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

                var pending = await _sessionService.GetPendingFeedbackAsync(sessionId);

                if (pending == null)
                    return NoContent();  // 204 = nothing pending, same contract as pending-lesson

                return Ok(pending);  // 200 with PendingFeedbackResponseDto
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving pending feedback for session {SessionId}", sessionId);
                return BadRequest("Its bad if you get this");
            }
        }

        /// <summary>
        /// Robot calls this after acting on an SLP feedback command to prevent re-execution.
        /// PUT /api/robotsessions/{sessionId}/pending-feedback/{feedbackId}/acknowledge
        /// </summary>
        [HttpPut("{sessionId}/pending-feedback/{feedbackId}/acknowledge")]
        public async Task<IActionResult> AcknowledgeFeedback(Guid sessionId, Guid feedbackId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

                await _sessionService.AcknowledgeFeedbackAsync(sessionId, feedbackId);

                return Ok(new
                {
                    Message = "Feedback acknowledged.",
                    SessionId = sessionId,
                    FeedbackId = feedbackId
                });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Feedback or session not found: {SessionId} {FeedbackId}", sessionId, feedbackId);
                return NotFound(new { Message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error acknowledging feedback {FeedbackId} for session {SessionId}", feedbackId, sessionId);
                return BadRequest("Its bad if you get this");
            }
        }
    }
}
