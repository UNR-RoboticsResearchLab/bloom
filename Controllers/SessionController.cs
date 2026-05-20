// bloom
// SessionController.cs
// Session lifecycle and real-time robot state management.

using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages robot session lifecycle and real-time state monitoring.
    /// Handles session creation (anonymous or authenticated), joining by 6-digit code,
    /// ending sessions, and robot membership within a session.
    /// Also owns robot state updates and maintains a historical record of state changes
    /// for post-session analysis.
    /// </summary>
    [Authorize]
    [ApiController]
    [Route("api/[controller]")]
    public class SessionController : BloomControllerBase
    {
        private readonly ILogger<SessionController> _logger;
        private readonly IRobotSessionService _sessionService;
        private readonly IRobotService _robotService;

        public SessionController(
            ILogger<SessionController> logger,
            IRobotSessionService sessionService,
            IRobotService robotService)
        {
            _logger = logger;
            _sessionService = sessionService;
            _robotService = robotService;
        }

        #region Session Management

        /// <summary>
        /// Creates a new robot session. Supports anonymous or authenticated callers.
        /// Returns the new session including its joinable 6-digit code.
        /// </summary>
        [HttpPost]
        public async Task<IActionResult> CreateSession([FromBody] StartSessionDto dto)
        {
            try
            {
                if (!ModelState.IsValid)
                    return BadRequest(ModelState);

                var robot = await _robotService.GetRobotByIdAsync(dto.RobotId);
                if (robot == null)
                    return NotFound(new { Message = $"Robot with ID {dto.RobotId} not found" });

                var userId = dto.Anonymous ? null : (dto.UserId ?? GetCurrentUserId());
                var session = await _sessionService.StartSessionAsync(dto.RobotId, userId, dto.Anonymous);
                var robotIds = (await _sessionService.GetSessionRobotsAsync(session.Id)).ToList();

                return CreatedAtAction(nameof(GetSession), new { sessionId = session.Id }, new RobotSessionResponseDto
                {
                    Id = session.Id,
                    UserId = session.UserId,
                    SessionCode = session.SessionCode,
                    CreatedAt = session.CreatedAt,
                    LastUpdatedAt = session.LastUpdatedAt,
                    Robots = session.Robots,
                    RobotIds = robotIds,
                    ActiveLessonId = session.ActiveLessonId
                });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error creating session for robot {RobotId}", dto.RobotId);
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns all sessions. Intended for admin use.
        /// </summary>
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
                        SessionCode = session.SessionCode,
                        CreatedAt = session.CreatedAt,
                        LastUpdatedAt = session.LastUpdatedAt,
                        Robots = session.Robots,
                        RobotIds = robotIds,
                        ActiveLessonId = session.ActiveLessonId
                    });
                }

                return Ok(sessionDtos);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving all sessions");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns a single session by ID, including its current robot list and active lesson.
        /// </summary>
        [HttpGet("{sessionId}")]
        public async Task<IActionResult> GetSession(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

                var robotIds = (await _sessionService.GetSessionRobotsAsync(sessionId)).ToList();

                return Ok(new RobotSessionResponseDto
                {
                    Id = session.Id,
                    UserId = session.UserId,
                    CreatedAt = session.CreatedAt,
                    LastUpdatedAt = session.LastUpdatedAt,
                    Robots = session.Robots,
                    RobotIds = robotIds,
                    ActiveLessonId = session.ActiveLessonId
                });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving session {SessionId}", sessionId);
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Looks up a session by its 6-digit code and claims it for the authenticated user if unclaimed.
        /// </summary>
        [HttpGet("join/{code}")]
        public async Task<IActionResult> JoinSession(string code)
        {
            try
            {
                var session = await _sessionService.GetSessionByCodeAsync(code);
                if (session == null)
                    return NotFound(new { Message = $"Session with code {code} not found" });

                var userId = GetCurrentUserId();
                if (userId != null && session.UserId == null)
                {
                    await _sessionService.SetSessionUserIdAsync(session.Id, userId);
                    session = await _sessionService.GetSessionAsync(session.Id) ?? session;
                }

                var robotIds = (await _sessionService.GetSessionRobotsAsync(session.Id)).ToList();

                return Ok(new RobotSessionResponseDto
                {
                    Id = session.Id,
                    UserId = session.UserId,
                    CreatedAt = session.CreatedAt,
                    LastUpdatedAt = session.LastUpdatedAt,
                    Robots = session.Robots,
                    RobotIds = robotIds,
                    ActiveLessonId = session.ActiveLessonId
                });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error joining session with code {Code}", code);
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Ends a session. Only the owning user may end their session.
        /// </summary>
        [HttpPost("{sessionId}/end")]
        public async Task<IActionResult> EndSession(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

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
                _logger.LogError(ex, "Error ending session {SessionId}", sessionId);
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Removes the user association from a session, making it unclaimed again.
        /// </summary>
        [HttpDelete("{sessionId}/user")]
        public async Task<IActionResult> UnpairUser(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            await _sessionService.ClearSessionUserAsync(sessionId);
            _logger.LogInformation("User unpaired from session {SessionId}", sessionId);
            return Ok(new { Message = "User unpaired successfully", SessionId = sessionId });
        }

        #endregion

        #region Robot Membership

        /// <summary>
        /// Adds an additional robot to an existing session. Only the session owner may add robots.
        /// </summary>
        [HttpPost("{sessionId}/robots")]
        public async Task<IActionResult> AddRobot(Guid sessionId, [FromBody] StartSessionDto dto)
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
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Removes a robot from a session. Only the session owner may remove robots.
        /// </summary>
        [HttpDelete("{sessionId}/robots/{robotId}")]
        public async Task<IActionResult> RemoveRobot(Guid sessionId, Guid robotId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

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
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns the IDs of all robots currently in the specified session.
        /// </summary>
        [HttpGet("{sessionId}/robots")]
        public async Task<IActionResult> GetSessionRobots(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

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
                return BadRequest(new { ex.Message });
            }
        }

        #endregion

        #region Robot State

        /// <summary>
        /// Updates the real-time state (status, task, behavior, speech) for a robot within the session.
        /// Each update is appended to the session's state history.
        /// </summary>
        [HttpPut("{sessionId}/robots/{robotId}/state")]
        public async Task<IActionResult> UpdateRobotState(Guid sessionId, Guid robotId, [FromBody] RobotStateDto dto)
        {
            try
            {
                if (!ModelState.IsValid)
                    return BadRequest(ModelState);

                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

                var currentUserId = GetCurrentUserId();
                if (currentUserId != null && session.UserId != null && session.UserId != currentUserId)
                {
                    _logger.LogWarning("User {UserId} attempted to update robot state in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

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
                return BadRequest(new { ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error updating robot state in session {SessionId}", sessionId);
                return BadRequest(new { message = $"Error updating robot state: {ex.Message}" });
            }
        }

        /// <summary>
        /// Returns the current state of every robot in the session.
        /// </summary>
        [HttpGet("{sessionId}/states")]
        public async Task<IActionResult> GetCurrentStates(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

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
                return BadRequest(new { message = $"Error retrieving robot states: {ex.Message}" });
            }
        }

        /// <summary>
        /// Returns the full chronological state-change history for the session. Intended for post-session review.
        /// </summary>
        [HttpGet("{sessionId}/history")]
        public async Task<IActionResult> GetSessionHistory(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

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
                return BadRequest(new { message = $"Error retrieving session history: {ex.Message}" });
            }
        }

        /// <summary>
        /// Returns all tracker events recorded during the session (e.g., engagement or attention events).
        /// </summary>
        [HttpGet("{sessionId}/tracker-events")]
        public async Task<IActionResult> GetTrackerEvents(Guid sessionId)
        {
            var events = await _sessionService.GetTrackerEventsAsync(sessionId);
            return Ok(events);
        }

        #endregion
    }
}
