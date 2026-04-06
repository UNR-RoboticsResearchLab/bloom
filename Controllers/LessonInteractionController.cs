using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Records and relays all interaction events during an active lesson session.
    /// The robot posts student interactions; the SLP posts approve/retry feedback commands.
    /// The robot polls for pending feedback and acknowledges each command after acting on it,
    /// completing the approve/retry round-trip through this controller.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class LessonInteractionController : ControllerBase
    {
        private readonly ILogger<LessonInteractionController> _logger;
        private readonly IRobotSessionService _sessionService;

        public LessonInteractionController(
            ILogger<LessonInteractionController> logger,
            IRobotSessionService sessionService)
        {
            _logger = logger;
            _sessionService = sessionService;
        }

        /// <summary>
        /// Endpoint for recording interactions during a lesson, such as student responses, timeouts, questions, etc.
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <param name="interaction">Details of the interaction to log</param>
        /// <returns>Result of the logging operation</returns>
        [HttpPost("{sessionId}")]
        public async Task<IActionResult> RecordLessonInteraction(
            string sessionId,
            [FromBody] LogLessonInteractionDto interaction)
        {
            try
            {
                if (!Guid.TryParse(sessionId, out var sessionGuid))
                    return BadRequest(new { message = "Invalid sessionId format." });

                var id = await _sessionService.LogLessonInteractionAsync(sessionGuid, interaction);
                return Ok(new { Message = "Interaction recorded.", InteractionId = id });
            }
            catch (KeyNotFoundException ex)
            {
                return NotFound(new { message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error recording lesson interaction for session {SessionId}", sessionId);
                return BadRequest(new { message = "An error occurred while recording the lesson interaction.", details = ex.Message });
            }
        }

        /// <summary>
        /// SLP posts approve or retry feedback for the current lesson step.
        /// POST /api/lessoninteractions/{sessionId}/feedback
        /// Body: { "stepId": 3, "feedbackCommand": "approve" }
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <param name="dto">Feedback command from SLP indicating whether to approve (advance
        [HttpPost("{sessionId}/feedback")]
        public async Task<IActionResult> RecordLessonFeedback(
            string sessionId,
            [FromBody] RecordSLPFeedbackDto dto)
        {
            if (!ModelState.IsValid)
            {
                return BadRequest(ModelState);
            }
            try
            {
                if (!Guid.TryParse(sessionId, out var sessionGuid))
                    return BadRequest(new { message = "Invalid sessionId format." });

                var feedbackId = await _sessionService.RecordSLPFeedbackAsync(sessionGuid, dto);

                return Ok(new
                {
                    Message = "SLP feedback recorded. Robot will receive on next poll.",
                    SessionId = sessionId,
                    FeedbackId = feedbackId,
                    FeedbackCommand = dto.FeedbackCommand
                });
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { message = ex.Message });
            }
            catch (ArgumentException ex)
            {
                _logger.LogWarning(ex, "Invalid feedback command for session {SessionId}", sessionId);
                return BadRequest(new { message = ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error recording SLP feedback for session {SessionId}", sessionId);
                return BadRequest(new { message = "An error occurred while recording the lesson feedback.", details = ex.Message });
            }
        }

        /// <summary>
        /// Robot polls this endpoint to check if an SLP has issued a feedback command
        /// (approve or retry) that has not yet been acknowledged.
        /// Returns 200 with command data, or 204 if no pending feedback.
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
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
                    return NoContent();

                return Ok(pending);
            }
            catch (KeyNotFoundException ex)
            {
                _logger.LogWarning(ex, "Session not found: {SessionId}", sessionId);
                return NotFound(new { ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving pending feedback for session {SessionId}", sessionId);
                return BadRequest(new { message = "An error occurred while retrieving pending feedback.", details = ex.Message });
            }
        }

        /// <summary>
        /// Robot calls this after acting on an SLP feedback command to prevent re-execution.
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <param name="feedbackId">ID of the feedback record to acknowledge</param>
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
                return NotFound(new { ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error acknowledging feedback {FeedbackId} for session {SessionId}", feedbackId, sessionId);
                return BadRequest(new { message = "An error occurred while acknowledging feedback.", details = ex.Message });
            }
        }
    }
}