// bloom
// LessonRuntimeController.cs
// All real-time lesson activity within an active session.
// Robot polling, robot reporting, SLP commands, and SLP feedback are co-located here.

using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Handles all real-time lesson activity within an active robot session.
    /// Robot polling: pending-lesson, step-control, feedback.
    /// Robot reporting: progress updates, interaction recording, acknowledgements.
    /// SLP commands: start, stop, skip, replay, set-step, feedback.
    /// </summary>
    [ApiController]
    [Route("api/lesson-runtime")]
    public class LessonRuntimeController : BloomControllerBase
    {
        private readonly ILogger<LessonRuntimeController> _logger;
        private readonly IRobotSessionService _sessionService;
        private readonly IStepControlService _stepControlService;
        private readonly IRepeatRequestDetector _repeatRequestDetector;

        public LessonRuntimeController(
            ILogger<LessonRuntimeController> logger,
            IRobotSessionService sessionService,
            IStepControlService stepControlService,
            IRepeatRequestDetector repeatRequestDetector)
        {
            _logger = logger;
            _sessionService = sessionService;
            _stepControlService = stepControlService;
            _repeatRequestDetector = repeatRequestDetector;
        }

        #region SLP Commands — lesson lifecycle

        /// <summary>
        /// SLP queues a lesson to start in the session. Robot picks it up on next poll.
        /// </summary>
        [HttpPost("{sessionId}/start")]
        public async Task<IActionResult> StartLesson(Guid sessionId, [FromBody] StartLessonDto dto)
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
                    _logger.LogWarning("User {UserId} attempted to start lesson in session owned by {SessionUserId}", currentUserId, session.UserId);
                    return Forbid();
                }

                var lessonSession = await _sessionService.StartLessonAsync(sessionId, dto);

                return Ok(new
                {
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
        /// SLP stops the active lesson. Robot exits the lesson and returns to idle.
        /// </summary>
        [HttpPost("{sessionId}/stop")]
        public async Task<IActionResult> StopLesson(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId != null)
                _stepControlService.SetPendingControl(sessionId, "stop");

            await _sessionService.StopLessonAsync(sessionId);
            _logger.LogInformation("Stop command issued for session {SessionId}", sessionId);
            return Ok(new { Message = "Stop command queued", SessionId = sessionId });
        }

        #endregion

        #region SLP Commands — step control

        /// <summary>
        /// SLP skips the current step. Robot picks up the command on its next poll.
        /// </summary>
        [HttpPost("{sessionId}/skip")]
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
        /// SLP replays the current step. Robot picks up the command on its next poll.
        /// </summary>
        [HttpPost("{sessionId}/replay")]
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
        /// SLP jumps to a specific step. Robot picks up the command on its next poll.
        /// </summary>
        [HttpPost("{sessionId}/set-step")]
        public async Task<IActionResult> SetStep(Guid sessionId, [FromBody] SetStepDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId == null)
                return BadRequest(new { Message = "No active lesson to set step on" });

            var totalSteps = await _sessionService.GetActiveLessonTotalStepsAsync(sessionId);
            if (dto.TargetStep < 1 || (totalSteps is int max && dto.TargetStep > max))
                return BadRequest(new { Message = $"TargetStep must be between 1 and {totalSteps ?? dto.TargetStep}" });

            _stepControlService.SetPendingControl(sessionId, "set_step", dto.TargetStep);
            _logger.LogInformation("Set step {TargetStep} command issued for session {SessionId}", dto.TargetStep, sessionId);
            return Ok(new { Message = "Set step command queued", SessionId = sessionId, dto.TargetStep });
        }

        /// <summary>
        /// SLP pauses the active lesson in place. Robot freezes on the current step
        /// on its next poll rather than exiting the lesson.
        /// </summary>
        [HttpPost("{sessionId}/pause")]
        public async Task<IActionResult> PauseLesson(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId == null)
                return BadRequest(new { Message = "No active lesson to pause" });

            _stepControlService.SetPendingControl(sessionId, "pause");
            _logger.LogInformation("Pause command issued for session {SessionId}", sessionId);
            return Ok(new { Message = "Pause command queued", SessionId = sessionId });
        }

        /// <summary>
        /// SLP resumes a paused lesson. Robot re-plays the step it was paused on.
        /// </summary>
        [HttpPost("{sessionId}/resume")]
        public async Task<IActionResult> ResumeLesson(Guid sessionId)
        {
            var session = await _sessionService.GetSessionAsync(sessionId);
            if (session == null)
                return NotFound(new { Message = $"Session with ID {sessionId} not found" });

            if (session.ActiveLessonId == null)
                return BadRequest(new { Message = "No active lesson to resume" });

            _stepControlService.SetPendingControl(sessionId, "resume");
            _logger.LogInformation("Resume command issued for session {SessionId}", sessionId);
            return Ok(new { Message = "Resume command queued", SessionId = sessionId });
        }

        /// <summary>
        /// SLP posts approve or retry feedback for the current lesson step.
        /// </summary>
        [HttpPost("{sessionId}/feedback")]
        public async Task<IActionResult> RecordFeedback(Guid sessionId, [FromBody] RecordSLPFeedbackDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            try
            {
                var feedbackId = await _sessionService.RecordSLPFeedbackAsync(sessionId, dto);
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

        #endregion

        #region Robot Polling — check for queued commands

        /// <summary>
        /// Robot polls for a lesson queued by the SLP. Returns 200 with lesson details or 204 if none.
        /// </summary>
        [HttpGet("{sessionId}/pending-lesson")]
        public async Task<IActionResult> GetPendingLesson(Guid sessionId)
        {
            try
            {
                var session = await _sessionService.GetSessionAsync(sessionId);
                if (session == null)
                    return NotFound(new { Message = $"Session with ID {sessionId} not found" });

                var pendingLesson = await _sessionService.GetPendingLessonAsync(sessionId);
                if (pendingLesson == null)
                    return NoContent();

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
        /// Robot polls for a pending step command (skip, replay, set_step, stop).
        /// Returns 200 with command or 204 if none.
        /// </summary>
        [HttpGet("{sessionId}/step-control")]
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
        /// Robot polls for an SLP approve/retry feedback command.
        /// Returns 200 with command or 204 if none.
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

        #endregion

        #region Robot Reporting — progress, interactions, acknowledgements

        /// <summary>
        /// Robot reports completed steps and current progress.
        /// </summary>
        [HttpPut("{sessionId}/progress")]
        public async Task<IActionResult> UpdateProgress(Guid sessionId, [FromBody] UpdateLessonProgressDto dto)
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
        /// Robot records a student interaction (response, timeout, question, etc.).
        /// </summary>
        [HttpPost("{sessionId}/interactions")]
        public async Task<IActionResult> RecordInteraction(Guid sessionId, [FromBody] LogLessonInteractionDto interaction)
        {
            try
            {
                var id = await _sessionService.LogLessonInteractionAsync(sessionId, interaction);

                // A student can ask the robot to repeat itself at any point in a
                // step, independent of whatever that step is actually waiting on
                // (an answer, a timeout, etc.) — so this is checked for every
                // interaction the robot reports, not just ones flagged as a
                // "Question"/"Response". This queues "repeat_last", NOT "replay":
                // "replay" is the SLP's deliberate "redo this whole step" command
                // (motor sequence, visual aid entrance, lead-in delay, then
                // script — see Back/Restart on the Controls page) and re-running
                // all of that for a simple "say that again" mid-prompt reads to
                // the student as the robot resetting rather than just repeating
                // itself. "repeat_last" re-speaks the current step's script only.
                var repeatRequested = _repeatRequestDetector.IsRepeatRequest(interaction.StudentResponse);
                if (repeatRequested)
                {
                    var session = await _sessionService.GetSessionAsync(sessionId);
                    if (session?.ActiveLessonId != null)
                    {
                        _stepControlService.SetPendingControl(sessionId, "repeat_last");
                        _logger.LogInformation(
                            "Detected repeat request in session {SessionId} (\"{StudentResponse}\"); queuing repeat_last",
                            sessionId, interaction.StudentResponse);
                    }
                    else
                    {
                        repeatRequested = false;
                    }
                }

                return Ok(new { Message = "Interaction recorded.", InteractionId = id, RepeatRequested = repeatRequested });
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
        /// Robot acknowledges a step control command after acting on it, clearing it server-side.
        /// </summary>
        [HttpDelete("{sessionId}/step-control")]
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
        /// Robot acknowledges an SLP feedback command after acting on it, preventing re-execution.
        /// </summary>
        [HttpPut("{sessionId}/feedback/{feedbackId}/acknowledge")]
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

        #endregion

        #region History / Review

        /// <summary>
        /// Get all lesson interactions recorded in a session. Intended for SLP/Admin review.
        /// </summary>
        [HttpGet("{sessionId}/interactions")]
        public async Task<IActionResult> GetInteractions(Guid sessionId)
        {
            var interactions = await _sessionService.GetLessonInteractionsAsync(sessionId);
            return Ok(interactions);
        }

        /// <summary>
        /// Get the lesson run history for a student. Each entry is one lesson run within a session.
        /// </summary>
        [HttpGet("student/{studentId}/history")]
        public async Task<IActionResult> GetStudentLessonHistory(string studentId)
        {
            if (string.IsNullOrWhiteSpace(studentId))
                return BadRequest(new { Message = "studentId is required" });

            var history = await _sessionService.GetStudentLessonHistoryAsync(studentId);
            return Ok(history);
        }

        #endregion
    }
}
