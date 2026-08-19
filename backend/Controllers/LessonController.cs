// bloom
// LessonController.cs
// Lesson content management and student progress tracking.

using System.Security.Claims;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages lesson content and delivery.
    /// Provides endpoints for creating lessons, retrieving lesson metadata and steps,
    /// and tracking student progress. Caller: SLP (content management) and Student (progress reads).
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class LessonController : BloomControllerBase
    {
        private readonly ILessonService _lessonService;
        private readonly ILessonProgressService _progressService;
        private readonly ILessonAiService _lessonAiService;
        private readonly IWebHostEnvironment _env;

        private static readonly string[] AllowedImageExtensions = [".png", ".jpg", ".jpeg", ".gif", ".webp", ".svg"];

        public LessonController(
            ILessonService lessonService,
            ILessonProgressService progressService,
            ILessonAiService lessonAiService,
            IWebHostEnvironment env)
        {
            _lessonService = lessonService;
            _progressService = progressService;
            _lessonAiService = lessonAiService;
            _env = env;
        }

        /// <summary>
        /// Returns metadata for all lessons (no step detail). Public endpoint.
        /// </summary>
        [HttpGet("all")]
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

        /// <summary>
        /// Returns full lesson data including all steps and interaction configs for the specified lesson.
        /// </summary>
        [HttpGet("{lessonId}")]
        public async Task<IActionResult> GetLessonInfo(string lessonId)
        {
            try
            {
                var lesson = await _lessonService.GetByIdAsync(lessonId);
                if (lesson == null)
                    return NotFound(new { message = "Lesson not found." });

                return Ok(ToDetailDto(lesson));
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        // Maps a full Lesson entity (with Steps/Behaviors/Interaction loaded) to the
        // detail DTO shape. Shared by GetLessonInfo and UpdateLesson so both return
        // identically-shaped lesson data.
        private static LessonDto ToDetailDto(Lesson lesson) => new()
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
                Title = s.Title,
                Type = s.Type,
                Script = s.Script,
                TimingSeconds = s.TimingSeconds,
                VisualAid = s.VisualAid,
                Behaviors = s.Behaviors != null ? new StepBehaviorsDto
                {
                    Behavior = s.Behaviors.Name,
                    FacialExpression = s.Behaviors.FacialExpression,
                    Gaze = s.Behaviors.Gaze,
                    HeadMovement = s.Behaviors.HeadMovement
                } : null,
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
        };

        /// <summary>
        /// Creates a new lesson authored by the authenticated user.
        /// </summary>
        [Authorize]
        [HttpPost("create")]
        public async Task<IActionResult> CreateLesson([FromBody] LessonDto lesson)
        {
            try
            {
                var userId = GetCurrentUserId();
                if (string.IsNullOrEmpty(userId))
                    return Unauthorized(new { message = "User not authenticated." });

                lesson.CreatedById = userId;
                var created = await _lessonService.CreateAsync(lesson);

                if (created != null)
                    // Keeps the historical `message` field (some older callers check
                    // response.message for "successfully") while also handing back the
                    // persisted lesson -- including its new Id -- for callers that need
                    // to act on it right away, e.g. running it on a robot.
                    return Ok(new { message = "Lesson created successfully.", lesson = ToDetailDto(created) });

                return BadRequest(new { message = "Failed to create lesson." });
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        /// <summary>
        /// Updates an existing lesson's info and replaces its full step list. Only the
        /// lesson's creator or an Admin may update it.
        /// </summary>
        [Authorize]
        [HttpPut("{lessonId}")]
        public async Task<IActionResult> UpdateLesson(string lessonId, [FromBody] LessonDto lesson)
        {
            try
            {
                var userId = GetCurrentUserId();
                if (string.IsNullOrEmpty(userId))
                    return Unauthorized(new { message = "User not authenticated." });

                if (!Guid.TryParse(lessonId, out _))
                    return BadRequest(new { message = "Invalid lesson id." });

                var existing = await _lessonService.GetByIdAsync(lessonId);
                if (existing == null)
                    return NotFound(new { message = "Lesson not found." });

                if (existing.CreatedById != userId && !User.IsInRole("Admin"))
                    return Forbid();

                lesson.Id = lessonId;
                var success = await _lessonService.UpdateAsync(lesson);

                if (!success)
                    return BadRequest(new { message = "Failed to update lesson." });

                var updated = await _lessonService.GetByIdAsync(lessonId);
                return Ok(updated != null ? ToDetailDto(updated) : new { message = "Lesson updated successfully." });
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        /// <summary>
        /// Generates a full lesson (title, objectives, steps) from a free-text prompt via AI.
        /// Returns a LessonDto for the caller to preview/edit in the builder — never persists directly.
        /// </summary>
        [Authorize]
        [HttpPost("ai/generate")]
        public async Task<IActionResult> GenerateLessonWithAi([FromBody] LessonAiGenerateRequestDto request)
        {
            try
            {
                var lesson = await _lessonAiService.GenerateLessonAsync(request);
                return Ok(lesson);
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Lesson generation failed: {ex.Message}" });
            }
        }

        /// <summary>
        /// Uploads an image to use as a step's visual aid and returns its relative URL.
        /// </summary>
        [Authorize]
        [HttpPost("steps/visual-aid")]
        [RequestSizeLimit(10_000_000)]
        public async Task<IActionResult> UploadVisualAid([FromForm] IFormFile file)
        {
            if (file == null || file.Length == 0)
                return BadRequest(new { message = "No file uploaded." });

            var ext = Path.GetExtension(file.FileName).ToLowerInvariant();
            if (!AllowedImageExtensions.Contains(ext) || !file.ContentType.StartsWith("image/"))
                return BadRequest(new { message = "Unsupported file type." });

            try
            {
                var webRoot = string.IsNullOrEmpty(_env.WebRootPath)
                    ? Path.Combine(_env.ContentRootPath, "wwwroot")
                    : _env.WebRootPath;
                var uploadsDir = Path.Combine(webRoot, "uploads", "lessons");
                Directory.CreateDirectory(uploadsDir);

                var fileName = $"{Guid.NewGuid()}{ext}";
                var filePath = Path.Combine(uploadsDir, fileName);

                using (var stream = new FileStream(filePath, FileMode.Create))
                {
                    await file.CopyToAsync(stream);
                }

                return Ok(new { url = $"/uploads/lessons/{fileName}" });
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Upload failed: {ex.Message}" });
            }
        }

        /// <summary>
        /// Removes a single step from a lesson.
        /// </summary>
        [Authorize]
        [HttpDelete("{lessonId}/steps/{stepId}")]
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

        /// <summary>
        /// Returns lesson progress records for the currently authenticated student.
        /// </summary>
        [Authorize]
        [HttpGet("progress/my")]
        public async Task<IActionResult> GetMyProgress()
        {
            var userId = GetCurrentUserId();
            if (string.IsNullOrEmpty(userId))
                return Unauthorized();

            var records = await _progressService.GetByUserIdWithLessonAsync(userId);
            var result = records.Select(p => new StudentLessonProgressDto
            {
                Id = p.Id,
                LessonId = p.LessonId,
                LessonTitle = p.Lesson?.Title ?? string.Empty,
                LessonStep = p.LessonStep,
                ProgressPercentage = p.ProgressPercentage,
                LastUpdated = p.LastUpdated
            });

            return Ok(result);
        }

        /// <summary>
        /// Returns lesson progress records for the specified student. Intended for SLP or admin callers.
        /// </summary>
        [Authorize]
        [HttpGet("progress/student/{studentId}")]
        public async Task<IActionResult> GetStudentProgress(string studentId)
        {
            var records = await _progressService.GetByUserIdWithLessonAsync(studentId);
            var result = records.Select(p => new StudentLessonProgressDto
            {
                Id = p.Id,
                LessonId = p.LessonId,
                LessonTitle = p.Lesson?.Title ?? string.Empty,
                LessonStep = p.LessonStep,
                ProgressPercentage = p.ProgressPercentage,
                LastUpdated = p.LastUpdated
            });

            return Ok(result);
        }
    }
}
