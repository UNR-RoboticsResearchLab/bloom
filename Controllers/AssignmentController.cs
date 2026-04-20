// bloom
// AssignmentController.cs
// API controller for managing lesson assignments to students

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
    /// Manages lesson assignments.
    /// SLPs create assignments linking a lesson to a student.
    /// Students can view their own assignments; SLPs can view any student's assignments.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    [Authorize]
    public class AssignmentController : ControllerBase
    {
        private readonly BloomDbContext _context;
        private readonly ILogger<AssignmentController> _logger;

        public AssignmentController(BloomDbContext context, ILogger<AssignmentController> logger)
        {
            _context = context;
            _logger = logger;
        }

        /// <summary>
        /// Create a lesson assignment for a student.
        /// </summary>
        [HttpPost]
        public async Task<IActionResult> CreateAssignment([FromBody] CreateAssignmentDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var assignedById = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (string.IsNullOrEmpty(assignedById))
                return Unauthorized();

            var lesson = await _context.Lessons.FindAsync(dto.LessonId);
            if (lesson == null)
                return NotFound(new { message = $"Lesson {dto.LessonId} not found." });

            var student = await _context.Accounts.FindAsync(dto.StudentId);
            if (student == null)
                return NotFound(new { message = $"Student {dto.StudentId} not found." });

            var assignment = new Assignment
            {
                StudentId = dto.StudentId,
                LessonId = dto.LessonId,
                AssignedById = assignedById,
                AssignedDate = DateTime.UtcNow,
                DueDate = dto.DueDate,
                IsCompleted = false
            };

            _context.Assignments.Add(assignment);
            await _context.SaveChangesAsync();

            _logger.LogInformation("Assignment {AssignmentId} created for student {StudentId}", assignment.Id, dto.StudentId);

            return Ok(new AssignmentResponseDto
            {
                Id = assignment.Id,
                StudentId = assignment.StudentId,
                LessonId = assignment.LessonId,
                LessonTitle = lesson.Title,
                AssignedById = assignment.AssignedById,
                AssignedDate = assignment.AssignedDate,
                DueDate = assignment.DueDate,
                IsCompleted = assignment.IsCompleted
            });
        }

        /// <summary>
        /// Get the authenticated student's own assignments.
        /// </summary>
        [HttpGet("my")]
        public async Task<IActionResult> GetMyAssignments()
        {
            var userId = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (string.IsNullOrEmpty(userId))
                return Unauthorized();

            var assignments = await _context.Assignments
                .Include(a => a.Lesson)
                .Where(a => a.StudentId == userId)
                .ToListAsync();

            return Ok(assignments.Select(ToDto));
        }

        /// <summary>
        /// Get all assignments for a specific student. Intended for SLP/Admin use.
        /// </summary>
        [HttpGet("student/{studentId}")]
        public async Task<IActionResult> GetStudentAssignments(string studentId)
        {
            var assignments = await _context.Assignments
                .Include(a => a.Lesson)
                .Where(a => a.StudentId == studentId)
                .ToListAsync();

            return Ok(assignments.Select(ToDto));
        }

        /// <summary>
        /// Manually mark an assignment as complete.
        /// </summary>
        [HttpPut("{id}/complete")]
        public async Task<IActionResult> CompleteAssignment(Guid id)
        {
            var assignment = await _context.Assignments.FindAsync(id);
            if (assignment == null)
                return NotFound(new { message = $"Assignment {id} not found." });

            assignment.IsCompleted = true;
            _context.Assignments.Update(assignment);
            await _context.SaveChangesAsync();

            return Ok(new { message = "Assignment marked as complete.", AssignmentId = id });
        }

        private static AssignmentResponseDto ToDto(Assignment a) => new()
        {
            Id = a.Id,
            StudentId = a.StudentId,
            LessonId = a.LessonId,
            LessonTitle = a.Lesson?.Title ?? string.Empty,
            AssignedById = a.AssignedById,
            AssignedDate = a.AssignedDate,
            DueDate = a.DueDate,
            IsCompleted = a.IsCompleted
        };
    }
}
