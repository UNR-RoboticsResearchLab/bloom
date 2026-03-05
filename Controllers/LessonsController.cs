using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class LessonsController : ControllerBase
    {

        private readonly IWebHostEnvironment _env;
        private readonly ILessonService _lessonService;
    
    
        //LessonController needs to handle getting lesson files and sending them to users
        public LessonsController(IWebHostEnvironment env, ILessonService lessonService)
        {
            _env = env;
            _lessonService = lessonService;
        }


        [HttpGet]
        [Route("{lessonId}")]
        public async Task<IActionResult> GetLessonInfo(string lessonId)
        {
            try
            {
                var lesson = await _lessonService.GetByIdAsync(lessonId);

                if (lesson == null)
                {
                    return NotFound(new { message = "Lesson not found." });
                }

                // Get and parse lesson file to verify it's valid JSON
                var lessonContent = await System.IO.File.ReadAllTextAsync(lesson.LessonFileUrl);
                try
                {
                    System.Text.Json.JsonDocument.Parse(lessonContent);
                }
                catch (System.Text.Json.JsonException)
                {
                    throw new ArgumentException("LessonDescription must be valid JSON.");
                }

                return Ok(new LessonDto
                {
                    Id = lesson.Id.ToString(),
                    Title = lesson.Title,
                    Description = lesson.Description,
                    CreatedDate = lesson.CreatedDate,
                    UpdatedDate = lesson.UpdatedDate,
                    LessonType = lesson.LessonType,
                    LessonDescription = lessonContent,
                    CreatedById = lesson.CreatedById
                });
            }
            catch(Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }

        [HttpPost]
        [Route("create")]
        public async Task<IActionResult> CreateLesson([FromBody] LessonDto lesson)
        {
            try
            {
                var success = await _lessonService.CreateAsync(lesson);
                
                if (success)
                {
                    return Ok(new { message = "Lesson created successfully." });
                }
                else
                {
                    return BadRequest(new { message = "Failed to create lesson." });
                }
            }
            catch (Exception ex)            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }




        // get json file and send file.
        [HttpPost]
        [Route("file")]
        public async Task<IActionResult> GetLessonFile([FromBody] string lessonId)
        {
            try
            {
                var lesson = await _lessonService.GetByIdAsync(lessonId);
                if (lesson == null)
                {
                    return NotFound();
                }
                var filePath = lesson.LessonFileUrl;

                if (!System.IO.File.Exists(filePath))
                {
                    return NotFound(new { message = "Lesson file not found." });
                }

                // Serve download
                var fileBytes = System.IO.File.ReadAllBytes(filePath);
                return File(fileBytes, "application/json", $"lesson-{lesson.Title}.json");
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = $"Request error: {ex.Message}" });
            }
        }



    }
}