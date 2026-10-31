// bloom
// UserController.cs
// User management, SLP client relationships, and lesson assignments.

using System.Security.Claims;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services.Accounts;
using bloom.Services.Email;
using bloom.Services.SLPClients;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.RateLimiting;
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
        private readonly IEmailConfirmationService _emailConfirmation;
        private readonly IPasswordResetService _passwordReset;

        public UserController(
            IAccountService accountService,
            ISLPClientService slpClientService,
            BloomDbContext context,
            ILogger<UserController> logger,
            IEmailConfirmationService emailConfirmation,
            IPasswordResetService passwordReset)
        {
            _accountService = accountService;
            _slpClientService = slpClientService;
            _context = context;
            _logger = logger;
            _emailConfirmation = emailConfirmation;
            _passwordReset = passwordReset;
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
            if (res.IsNotAllowed)
                return BadRequest(new
                {
                    Message = "Email not confirmed. Check your inbox for the confirmation link, or request a new one.",
                    Code = "EmailNotConfirmed"
                });

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

            // Public sign-up must never mint an Admin: Admin grants fleet-wide remote shell access.
            // Only an already-authenticated Admin may create another one.
            if (string.Equals(account?.SelectedRole, "ADMIN", StringComparison.OrdinalIgnoreCase)
                && !User.IsInRole("Admin"))
                return StatusCode(StatusCodes.Status403Forbidden,
                    new { Message = "Only an administrator can create administrator accounts." });

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

            // The account exists either way; if mail fails the user can request another link.
            var emailSent = true;
            try
            {
                await _emailConfirmation.SendConfirmationAsync(newUser);
            }
            catch (Exception ex)
            {
                emailSent = false;
                _logger.LogError(ex, "Could not send confirmation email for account {UserId}", newUser.Id);
            }

            return Ok(new
            {
                Message = emailSent
                    ? "Creation Success. Check your email to confirm your account."
                    : "Creation Success, but the confirmation email could not be sent. Request a new one from the sign-in page.",
                EmailSent = emailSent,
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
        /// Confirms an email address from the link in the confirmation email. Returns an HTML page.
        /// </summary>
        [HttpGet("confirm-email")]
        public async Task<ContentResult> ConfirmEmail([FromQuery] string? userId, [FromQuery] string? token)
        {
            var confirmed = await _emailConfirmation.ConfirmEmailAsync(userId ?? "", token ?? "");
            return Content(confirmed
                ? ConfirmationPage("Email confirmed", "Thanks! Your email address is confirmed and you can now sign in to Bloom.")
                : ConfirmationPage("Confirmation failed", "This confirmation link is invalid or has expired. Sign in to Bloom to request a new one."),
                "text/html");
        }

        /// <summary>
        /// Sends a new confirmation email if the address belongs to an unconfirmed account.
        /// Always answers the same so it cannot be used to discover registered addresses.
        /// </summary>
        [HttpPost("resend-confirmation")]
        [EnableRateLimiting("resend-confirmation")]
        public async Task<IActionResult> ResendConfirmation([FromBody] ResendConfirmationDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            try
            {
                await _emailConfirmation.ResendConfirmationAsync(dto.Email);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Could not resend confirmation email");
            }

            return Ok(new { Message = "If that address belongs to an unconfirmed account, a new confirmation email has been sent." });
        }

        /// <summary>
        /// Emails a password reset link. Always answers the same so it cannot be used to discover
        /// registered addresses.
        /// </summary>
        [HttpPost("forgot-password")]
        [EnableRateLimiting("forgot-password")]
        public async Task<IActionResult> ForgotPassword([FromBody] ForgotPasswordDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            try
            {
                await _passwordReset.SendResetLinkAsync(dto.Email);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Could not send password reset email");
            }

            return Ok(new { Message = "If that address belongs to an account, an email with instructions has been sent." });
        }

        /// <summary>
        /// The page the reset link opens: a form for choosing a new password.
        /// </summary>
        [HttpGet("reset-password")]
        public ContentResult ResetPasswordForm([FromQuery] string? userId, [FromQuery] string? token) =>
            Content(ResetPasswordPage(userId ?? "", token ?? "", null), "text/html");

        /// <summary>
        /// Submitted by the reset form.
        /// </summary>
        [HttpPost("reset-password")]
        [Consumes("application/x-www-form-urlencoded")]
        public async Task<ContentResult> ResetPassword([FromForm] string? userId, [FromForm] string? token, [FromForm] string? password)
        {
            if (string.IsNullOrEmpty(password))
                return Content(ResetPasswordPage(userId ?? "", token ?? "", "Enter a new password."), "text/html");

            var result = await _passwordReset.ResetPasswordAsync(userId ?? "", token ?? "", password);
            if (!result.Succeeded)
                return Content(ResetPasswordPage(userId ?? "", token ?? "",
                    string.Join(" ", result.Errors.Select(e => e.Description))), "text/html");

            return Content(ConfirmationPage("Password changed", "Your password has been updated. You can now sign in to Bloom."), "text/html");
        }

        private static string ResetPasswordPage(string userId, string token, string? error)
        {
            Func<string, string> enc = System.Net.WebUtility.HtmlEncode;
            var errorHtml = error == null ? "" : $"<p style=\"color:#b91c1c\">{enc(error)}</p>";
            return "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">" +
                "<title>Reset password - Bloom</title></head>" +
                "<body style=\"font-family:sans-serif;max-width:24rem;margin:6rem auto;padding:0 1rem;text-align:center\">" +
                $"<h1>Choose a new password</h1>{errorHtml}" +
                "<form method=\"post\" action=\"/api/user/reset-password\">" +
                $"<input type=\"hidden\" name=\"userId\" value=\"{enc(userId)}\">" +
                $"<input type=\"hidden\" name=\"token\" value=\"{enc(token)}\">" +
                "<p><input type=\"password\" name=\"password\" placeholder=\"New password\" autocomplete=\"new-password\" required style=\"width:100%;padding:.5rem;box-sizing:border-box\"></p>" +
                "<p><button type=\"submit\" style=\"padding:.5rem 1rem\">Change password</button></p></form></body></html>";
        }

        private static string ConfirmationPage(string title, string message) =>
            "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">" +
            $"<title>{title} - Bloom</title></head>" +
            "<body style=\"font-family:sans-serif;max-width:32rem;margin:6rem auto;padding:0 1rem;text-align:center\">" +
            $"<h1>{title}</h1><p>{message}</p><p><a href=\"/sign-in\">Go to sign in</a></p></body></html>";

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

            // The student must confirm the address before they can sign in with the temporary password.
            var emailSent = true;
            try
            {
                await _emailConfirmation.SendConfirmationAsync(newUser, generatedPassword);
            }
            catch (Exception ex)
            {
                emailSent = false;
                _logger.LogError(ex, "Could not send confirmation email for student {UserId}", newUser.Id);
            }

            return Ok(new
            {
                Message = emailSent
                    ? "Student account created successfully. A confirmation email with the temporary password was sent to the student."
                    : "Student account created, but the confirmation email could not be sent. The student can request a new one.",
                EmailSent = emailSent,
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
