using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class LessonInteractionsController : ControllerBase
    {
        private readonly ILogger<LessonInteractionsController> _logger;
        private readonly IRobotSessionService _sessionService;

        public LessonInteractionsController(
            ILogger<LessonInteractionsController> logger,
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
    }
}