// bloom
// LessonController.cs
// API controller for managing lesson content and delivery.

using System.Security.Claims;
using System.Text;
using System.Text.Json;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages lesson content and delivery.
    /// Provides endpoints for creating lessons, retrieving lesson metadata and steps,
    /// serving lesson JSON for robot consumption, and removing individual steps.
    /// Lesson content is stored in the database; file export reconstructs the original format on demand.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class LessonController : ControllerBase
    {
        private readonly ILessonService _lessonService;

        public LessonController(ILessonService lessonService)
        {
            _lessonService = lessonService;
        }

        [HttpGet]
        [Route("all")]
        public async Task<IActionResult> GetAllLessons()
        {
            try
            {
                var lessons = await _lessonService.GetAllAsync();
                var lessonDtos = lessons.Select(lesson => new LessonDto
                {
                    Id = lesson.Id.ToString(),
                    Title = lesson.Title,
                    Description = lesson.Description,
                    CreatedDate = lesson.CreatedDate,
                    UpdatedDate = lesson.UpdatedDate,
                    LessonType = lesson.LessonType,
                    CreatedById = lesson.CreatedById,
                    LearningObjectives = lesson.LearningObjectives
                });
                return Ok(lessonDtos);
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        [HttpGet]
        [Route("{lessonId}")]
        public async Task<IActionResult> GetLessonInfo(string lessonId)
        {
            try
            {
                var lesson = await _lessonService.GetByIdAsync(lessonId);

                if (lesson == null)
                    return NotFound(new { message = "Lesson not found." });

                return Ok(new LessonDto
                {
                    Id = lesson.Id.ToString(),
                    Title = lesson.Title,
                    Description = lesson.Description,
                    CreatedDate = lesson.CreatedDate,
                    UpdatedDate = lesson.UpdatedDate,
                    LessonType = lesson.LessonType,
                    CreatedById = lesson.CreatedById,
                    LearningObjectives = lesson.LearningObjectives,
                    Steps = [.. lesson.Steps.Select(s => new LessonStepDto
                    {
                        Id = s.Id,
                        StepOrder = s.StepOrder,
                        Type = s.Type,
                        Script = s.Script,
                        TimingSeconds = s.TimingSeconds,
                        VisualAid = s.VisualAid,
                        Behaviors = s.Behaviors,
                        Interaction = s.Interaction != null ? new StepInteractionDto
                        {
                            Id = s.Interaction.Id,
                            WaitForResponse = s.Interaction.WaitForResponse,
                            MaxWaitSeconds = s.Interaction.MaxWaitSeconds,
                            CorrectAnswer = s.Interaction.CorrectAnswer,
                            CorrectResponseScript = s.Interaction.CorrectResponseScript,
                            IncorrectResponseScript = s.Interaction.IncorrectResponseScript,
                            SingleTurnLlm = s.Interaction.SingleTurnLlm,
                            SingleTurnLlmPrompt = s.Interaction.SingleTurnLlmPrompt,
                            LlmFollowUp = s.Interaction.LlmFollowUp,
                            FallbackScript = s.Interaction.FallbackScript,
                            FallbackVisualAid = s.Interaction.FallbackVisualAid,
                            FallbackVisualAidLabels = s.Interaction.FallbackVisualAidLabels,
                        } : null
                    })]
                });
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        [Authorize]
        [HttpPost]
        [Route("create")]
        public async Task<IActionResult> CreateLesson([FromBody] LessonDto lesson)
        {
            try
            {
                var userId = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
                if (string.IsNullOrEmpty(userId))
                    return Unauthorized(new { message = "User not authenticated." });

                lesson.CreatedById = userId;

                var success = await _lessonService.CreateAsync(lesson);

                if (success)
                    return Ok(new { message = "Lesson created successfully." });

                return BadRequest(new { message = "Failed to create lesson." });
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        [Authorize]
        [HttpDelete]
        [Route("{lessonId}/steps/{stepId}")]
        public async Task<IActionResult> DeleteStep(Guid lessonId, Guid stepId)
        {
            try
            {
                var success = await _lessonService.RemoveStepAsync(lessonId, stepId);

                if (!success)
                    return NotFound(new { message = "Step not found." });

                return Ok(new { message = "Step removed successfully.", LessonId = lessonId, StepId = stepId });
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }
    }
}