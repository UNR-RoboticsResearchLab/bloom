// bloom
// SLPClientController.cs
// API controller for managing SLPClient records (student-SLP associations)

using System.Security.Claims;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages SLPClient records, which associate a student with one or more SLP teachers.
    /// SLPs create client profiles, view their clients, and retrieve progress for each student.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    [Authorize]
    public class SLPClientController : ControllerBase
    {
        private readonly BloomDbContext _context;
        private readonly ILogger<SLPClientController> _logger;

        public SLPClientController(BloomDbContext context, ILogger<SLPClientController> logger)
        {
            _context = context;
            _logger = logger;
        }

        /// <summary>
        /// Create a new SLPClient profile, linking a student to the requesting SLP.
        /// The requesting SLP is automatically added as a teacher on the client.
        /// </summary>
        [HttpPost]
        public async Task<IActionResult> CreateClient([FromBody] CreateSLPClientDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var slpId = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var student = await _context.Accounts.FindAsync(dto.StudentId);
            if (student == null)
                return NotFound(new { message = $"Student {dto.StudentId} not found." });

            var slp = await _context.Accounts.FindAsync(slpId);
            if (slp == null)
                return Unauthorized();

            var client = new SLPClient
            {
                Name = dto.Name,
                StudentId = dto.StudentId,
                CreatedDate = DateTime.UtcNow,
                Teachers = new List<Account> { slp }
            };

            _context.SLPClients.Add(client);
            await _context.SaveChangesAsync();

            _logger.LogInformation("SLPClient {ClientId} created for student {StudentId} by SLP {SlpId}", client.Id, dto.StudentId, slpId);

            return Ok(new SLPClientResponseDto
            {
                Id = client.Id,
                Name = client.Name,
                StudentId = client.StudentId,
                StudentName = student.FullName
            });
        }

        /// <summary>
        /// List all SLPClients where the authenticated SLP is a teacher.
        /// </summary>
        [HttpGet]
        public async Task<IActionResult> GetMyClients()
        {
            var slpId = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var clients = await _context.SLPClients
                .Include(c => c.Student)
                .Include(c => c.Teachers)
                .ToListAsync();

            var filtered = clients
                .Where(c => c.Teachers != null && c.Teachers.Any(t => t.Id == slpId))
                .Select(c => new SLPClientResponseDto
                {
                    Id = c.Id,
                    Name = c.Name,
                    StudentId = c.StudentId,
                    StudentName = c.Student?.FullName
                })
                .ToList();

            return Ok(filtered);
        }

        /// <summary>
        /// Get full details for an SLPClient, including the student's assignments and lesson progress.
        /// </summary>
        [HttpGet("{id}")]
        public async Task<IActionResult> GetClient(Guid id)
        {
            var client = await _context.SLPClients
                .Include(c => c.Student)
                .Include(c => c.Teachers)
                .Include(c => c.Assignments)
                    !.ThenInclude(a => a!.Lesson)
                .FirstOrDefaultAsync(c => c.Id == id);

            if (client == null)
                return NotFound(new { message = $"SLPClient {id} not found." });

            var progress = await _context.LessonProgresses
                .Include(p => p.Lesson)
                .Where(p => p.StudentId == client.StudentId)
                .ToListAsync();

            return Ok(new SLPClientResponseDto
            {
                Id = client.Id,
                Name = client.Name,
                StudentId = client.StudentId,
                StudentName = client.Student?.FullName,
                Assignments = (client.Assignments ?? []).Select(a => new AssignmentResponseDto
                {
                    Id = a.Id,
                    StudentId = a.StudentId,
                    LessonId = a.LessonId,
                    LessonTitle = a.Lesson?.Title ?? string.Empty,
                    AssignedById = a.AssignedById,
                    AssignedDate = a.AssignedDate,
                    DueDate = a.DueDate,
                    IsCompleted = a.IsCompleted
                }).ToList(),
                Progress = progress.Select(p => new StudentLessonProgressDto
                {
                    Id = p.Id,
                    LessonId = p.LessonId,
                    LessonTitle = p.Lesson?.Title ?? string.Empty,
                    LessonStep = p.LessonStep,
                    ProgressPercentage = p.ProgressPercentage,
                    LastUpdated = p.LastUpdated
                }).ToList()
            });
        }

        /// <summary>
        /// Add another SLP as a teacher on an existing client.
        /// </summary>
        [HttpPost("{id}/teachers/{teacherId}")]
        public async Task<IActionResult> AddTeacher(Guid id, string teacherId)
        {
            var client = await _context.SLPClients
                .Include(c => c.Teachers)
                .FirstOrDefaultAsync(c => c.Id == id);

            if (client == null)
                return NotFound(new { message = $"SLPClient {id} not found." });

            var teacher = await _context.Accounts.FindAsync(teacherId);
            if (teacher == null)
                return NotFound(new { message = $"Account {teacherId} not found." });

            client.Teachers ??= new List<Account>();

            if (client.Teachers.Any(t => t.Id == teacherId))
                return Conflict(new { message = "Teacher already associated with this client." });

            client.Teachers.Add(teacher);
            await _context.SaveChangesAsync();

            return Ok(new { message = "Teacher added.", ClientId = id, TeacherId = teacherId });
        }
    }
}
