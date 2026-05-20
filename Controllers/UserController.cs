// bloom
// UserController.cs
// User management, SLP client relationships, and lesson assignments.

using System.Security.Claims;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages accounts, SLP-student relationships, and lesson assignments.
    /// Auth: login and account creation (public); profile mutations (authorized).
    /// SLP clients: SLP creates and manages student relationships.
    /// Assignments: SLP creates assignments; students view their own.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class UserController : BloomControllerBase
    {
        private readonly IAccountService _accountService;
        private readonly ISLPClientService _slpClientService;
        private readonly BloomDbContext _context;
        private readonly ILogger<UserController> _logger;

        public UserController(
            IAccountService accountService,
            ISLPClientService slpClientService,
            BloomDbContext context,
            ILogger<UserController> logger)
        {
            _accountService = accountService;
            _slpClientService = slpClientService;
            _context = context;
            _logger = logger;
        }

        #region Auth


        /// <summary>
        ///  Logs in an existing user.
        /// </summary>
        /// <param name="account"></param>
        /// <returns>a cookie</returns>
        [HttpPost("login")]
        public async Task<IActionResult> Login([FromBody] LoginDto account)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            if (account.Email == null || account.Password == null)
                return BadRequest(new { Message = "Email and Password are required." });

            var res = await _accountService.SignInAsync(account.Email, account.Password);
            if (!res.Succeeded)
                return BadRequest(new { Message = "Invalid login attempt." });

            var user = await _accountService.GetByEmailAsync(account.Email);
            if (user == null)
                return BadRequest(new { Message = "User not found." });

            return Ok(new
            {
                Message = "Login successful",
                User = new
                {
                    user.Id,
                    user.UserName,
                    user.Email,
                    user.FullName,
                    user.EmailConfirmed,
                    user.Role
                }
            });
        }

        /// <summary>
        /// Create a new user.
        /// </summary>
        /// <param name="account"></param>
        /// <returns></returns>
        [HttpPost("create")]
        public async Task<IActionResult> Create([FromBody] CreateAccountDto account)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var result = account?.SelectedRole.ToUpper() switch
            {
                "ADMIN" => await _accountService.RegisterAdminAsync(account),
                "FACILITATOR" => await _accountService.RegisterFacilitatorAsync(account),
                "STUDENT" => await _accountService.RegisterStudentAsync(account),
                "TEACHER" => await _accountService.RegisterTeacherAsync(account),
                "SLP" => await _accountService.RegisterSLPAsync(account),
                _ => null
            };

            if (result == null)
                return BadRequest(new { Message = "Invalid role specified." });

            if (!result.Succeeded)
            {
                foreach (var error in result.Errors)
                    return BadRequest(new { Message = error.Description });
                return BadRequest();
            }

            var newUser = await _accountService.GetByEmailAsync(account!.Email);
            if (newUser == null)
                return BadRequest(new { Message = "An error occurred during account creation." });

            return Ok(new
            {
                Message = "Creation Success",
                User = new
                {
                    newUser.Id,
                    newUser.FullName,
                    newUser.UserName,
                    newUser.Email,
                    newUser.EmailConfirmed,
                    newUser.Role
                }
            });
        }

        /// <summary>
        /// Creates a student account with a system-generated password and automatically links them as an SLP client.
        /// </summary>
        [Authorize]
        [HttpPost("student")]
        public async Task<IActionResult> CreateStudent([FromBody] CreateStudentByPrivilegedUserDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var (result, generatedPassword) = await _accountService.RegisterStudentWithGeneratedPasswordAsync(dto);
            if (!result.Succeeded)
            {
                foreach (var error in result.Errors)
                    return BadRequest(new { Message = error.Description });
                return BadRequest();
            }

            var newUser = await _accountService.GetByEmailAsync(dto.Email);
            if (newUser == null)
                return BadRequest(new { Message = "An error occurred during account creation." });

            var slpId = GetCurrentUserId();
            if (slpId != null)
                await _slpClientService.CreateAsync(newUser.FullName ?? newUser.Email ?? newUser.Id, slpId, newUser.Id);

            return Ok(new
            {
                Message = "Student account created successfully.",
                User = new
                {
                    newUser.Id,
                    newUser.FullName,
                    newUser.Email,
                    newUser.Role
                },
                TemporaryPassword = generatedPassword
            });
        }


        /// <summary>
        /// Get user profile by id.
        /// </summary>
        /// <param name="id"></param>
        /// <returns>A user dto</returns>
        [Authorize]
        [HttpGet("{id}")]
        public async Task<IActionResult> GetProfile(string id)
        {
            var user = await _accountService.GetByIdAsync(id);
            if (user == null)
                return BadRequest(new { Message = "User not found." });

            return Ok(new
            {
                user.Id,
                user.FullName,
                user.UserName,
                user.Email,
                user.EmailConfirmed,
                user.Role,
                RegisteredRobots = user.RegisteredRobots,
                AssignedAssignments = user.AssignedAssignments,
                Lessons = user.CreatedLessons
            });
        }

        /// <summary>
        /// Get currently logged in user.
        /// </summary>
        [Authorize]
        [HttpGet("me")]
        public async Task<IActionResult> GetCurrentUser()
        {
            var userId = GetCurrentUserId();

            if (userId == null)
            {
                return BadRequest(new { Message = "The current user could not be found."});
            }

            var user = await _accountService.GetByIdAsync(userId);

            if (user == null)
            {
                return BadRequest(new { Message = "The current user could not be found."});
            }

            return Ok(new
            {
                user.Id,
                user.FullName,
                user.UserName,
                user.Email,
                user.EmailConfirmed,
                user.Role,
                RegisteredRobots = user.RegisteredRobots,
                AssignedAssignments = user.AssignedAssignments,
                Lessons = user.CreatedLessons
            });
        }


        /// <summary>
        /// Updates name, username, or email for the specified user account.
        /// </summary>
        [Authorize]
        [HttpPut("{id}")]
        public async Task<IActionResult> UpdateProfile(string id, [FromBody] UpdateAccountDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var user = await _accountService.GetByIdAsync(id);
            if (user == null)
                return BadRequest(new { Message = "User not found." });

            user.FullName = dto.FullName ?? user.FullName;
            user.UserName = dto.UserName ?? user.UserName;
            user.Email = dto.Email ?? user.Email;

            var result = await _accountService.UpdateAsync(user);
            if (!result.Succeeded)
                return BadRequest(new { Message = "Failed to update user." });

            return Ok(new
            {
                Message = "User updated successfully.",
                User = new
                {
                    user.Id,
                    user.FullName,
                    user.UserName,
                    user.Email,
                    user.Role
                }
            });
        }

        /// <summary>
        /// Permanently deletes the specified user account.
        /// </summary>
        [Authorize]
        [HttpDelete("{id}")]
        public async Task<IActionResult> DeleteProfile(string id)
        {
            var user = await _accountService.GetByIdAsync(id);
            if (user == null)
                return BadRequest(new { Message = "User not found." });

            var result = await _accountService.DeleteAsync(user);
            if (!result.Succeeded)
                return BadRequest(new { Message = "Failed to delete user." });

            return Ok(new { Message = "User deleted successfully." });
        }

        #endregion

        #region SLP Clients

        /// <summary>
        /// Creates an SLP-client relationship between the current SLP and an existing student account.
        /// </summary>
        [Authorize]
        [HttpPost("clients")]
        public async Task<IActionResult> CreateClient([FromBody] CreateSLPClientDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var slpId = GetCurrentUserId();
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var result = await _slpClientService.CreateClientAsync(dto.Name, slpId, dto.StudentId);
            if (result == null)
                return NotFound(new { message = $"Student {dto.StudentId} not found." });

            _logger.LogInformation("SLPClient {ClientId} created for student {StudentId} by SLP {SlpId}", result.Id, dto.StudentId, slpId);
            return Ok(result);
        }

        /// <summary>
        /// Returns all SLP clients belonging to the currently authenticated SLP.
        /// </summary>
        [Authorize]
        [HttpGet("clients")]
        public async Task<IActionResult> GetMyClients()
        {
            var slpId = GetCurrentUserId();
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var clients = await _slpClientService.GetClientsForSlpAsync(slpId);
            return Ok(clients);
        }

        /// <summary>
        /// Returns a single SLP client record by ID.
        /// </summary>
        [Authorize]
        [HttpGet("clients/{id}")]
        public async Task<IActionResult> GetClient(Guid id)
        {
            var client = await _slpClientService.GetClientAsync(id);
            if (client == null)
                return NotFound(new { message = $"SLPClient {id} not found." });

            return Ok(client);
        }

        /// <summary>
        /// Removes an SLP-client relationship. Only the owning SLP may delete their own client records.
        /// </summary>
        [Authorize]
        [HttpDelete("clients/{id}")]
        public async Task<IActionResult> DeleteClient(Guid id)
        {
            var slpId = GetCurrentUserId();
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var result = await _slpClientService.DeleteClientAsync(id, slpId);
            return result switch
            {
                DeleteClientResult.Deleted => Ok(new { message = "SLPClient deleted.", ClientId = id }),
                DeleteClientResult.NotFound => NotFound(new { message = $"SLPClient {id} not found." }),
                DeleteClientResult.Forbidden => Forbid(),
                _ => BadRequest(new { message = "Failed to delete SLPClient." })
            };
        }

        #endregion

        #region Assignments

        /// <summary>
        /// Assigns a lesson to a student. Caller must be authenticated; lesson and student must exist.
        /// </summary>
        [Authorize]
        [HttpPost("assignments")]
        public async Task<IActionResult> CreateAssignment([FromBody] CreateAssignmentDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var assignedById = GetCurrentUserId();
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
        /// Returns all assignments for the currently authenticated student.
        /// </summary>
        [Authorize]
        [HttpGet("assignments/my")]
        public async Task<IActionResult> GetMyAssignments()
        {
            var userId = GetCurrentUserId();
            if (string.IsNullOrEmpty(userId))
                return Unauthorized();

            var assignments = await _context.Assignments
                .Include(a => a.Lesson)
                .Where(a => a.StudentId == userId)
                .ToListAsync();

            return Ok(assignments.Select(ToAssignmentDto));
        }

        /// <summary>
        /// Returns all assignments for the specified student. Intended for SLP or admin callers.
        /// </summary>
        [Authorize]
        [HttpGet("assignments/student/{studentId}")]
        public async Task<IActionResult> GetStudentAssignments(string studentId)
        {
            var assignments = await _context.Assignments
                .Include(a => a.Lesson)
                .Where(a => a.StudentId == studentId)
                .ToListAsync();

            return Ok(assignments.Select(ToAssignmentDto));
        }

        /// <summary>
        /// Marks the specified assignment as completed.
        /// </summary>
        [Authorize]
        [HttpPut("assignments/{id}/complete")]
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

        private static AssignmentResponseDto ToAssignmentDto(Assignment a) => new()
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

        #endregion
    }
}
