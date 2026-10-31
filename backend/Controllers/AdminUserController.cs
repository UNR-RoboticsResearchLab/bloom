// bloom
// AdminUserController.cs
// Admin-only user and role management: list accounts, change a role, delete an account.
// Creating accounts stays with POST api/user/create, which already lets an Admin create any role.

using bloom.Models.dto;
using bloom.Services.Accounts;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;

namespace bloom.Controllers
{
    /// <summary>
    /// Admin = remote shell on every robot, so this is cookie-only like the rest of fleet admin
    /// (see the "AdminOnly" policy in Program.cs).
    /// </summary>
    [ApiController]
    [Route("api/admin/users")]
    [Authorize(Policy = "AdminOnly")]
    public class AdminUserController : BloomControllerBase
    {
        // Roles an admin can assign. Participant is left out: those accounts sign in with a
        // ParticipantCode and are created by the ArSr study flow.
        public static readonly string[] AssignableRoles =
            { "Admin", "SLP", "Teacher", "Facilitator", "Student", "Researcher" };

        private const string Admin = "Admin";

        private readonly IAccountService _accounts;
        private readonly ILogger<AdminUserController> _logger;

        public AdminUserController(IAccountService accounts, ILogger<AdminUserController> logger)
        {
            _accounts = accounts;
            _logger = logger;
        }

        /// <summary>All accounts with their Identity role.</summary>
        [HttpGet]
        public async Task<ActionResult<IEnumerable<AdminUserDto>>> List()
        {
            var rows = await _accounts.GetAllWithRolesAsync();
            return Ok(rows.Select(r => new AdminUserDto
            {
                Id = r.Account.Id,
                FullName = r.Account.FullName,
                Email = r.Account.Email,
                UserName = r.Account.UserName,
                Role = r.Roles.FirstOrDefault() ?? r.Account.Role,
                CreatedDate = r.Account.CreatedDate,
            }));
        }

        /// <summary>The roles an admin may assign, for the role picker.</summary>
        [HttpGet("roles")]
        public ActionResult<IEnumerable<string>> Roles() => Ok(AssignableRoles);

        /// <summary>Replaces the user's role. Can't change your own, or demote the last Admin.</summary>
        [HttpPut("{id}/role")]
        public async Task<IActionResult> SetRole(string id, [FromBody] SetUserRoleDto dto)
        {
            var role = AssignableRoles.FirstOrDefault(r => string.Equals(r, dto.Role, StringComparison.OrdinalIgnoreCase));
            if (role == null)
                return BadRequest(new { Message = $"Role must be one of: {string.Join(", ", AssignableRoles)}." });

            if (id == GetCurrentUserId())
                return Conflict(new { Message = "You can't change your own role." });

            var user = await FindAsync(id);
            if (user == null) return NotFound(new { Message = "User not found." });

            if (!string.IsNullOrEmpty(user.ParticipantCode))
                return Conflict(new { Message = "Study participant accounts can't be given another role." });

            var wasAdmin = await _accounts.IsInRoleAsync(user, Admin);
            if (wasAdmin && role != Admin && await _accounts.CountInRoleAsync(Admin) <= 1)
                return Conflict(new { Message = "That's the last administrator." });

            var result = await _accounts.SetRoleAsync(user, role);
            if (!result.Succeeded)
                return BadRequest(new { Message = result.Errors.FirstOrDefault()?.Description ?? "Failed to change role." });

            _logger.LogWarning("User {UserId} role set to {Role} by {AdminId}", id, role, GetCurrentUserId());
            return Ok(new { user.Id, Role = role });
        }

        /// <summary>Permanently deletes an account. Can't delete yourself or the last Admin.</summary>
        [HttpDelete("{id}")]
        public async Task<IActionResult> Delete(string id)
        {
            if (id == GetCurrentUserId())
                return Conflict(new { Message = "You can't delete your own account." });

            var user = await FindAsync(id);
            if (user == null) return NotFound(new { Message = "User not found." });

            if (await _accounts.IsInRoleAsync(user, Admin) && await _accounts.CountInRoleAsync(Admin) <= 1)
                return Conflict(new { Message = "That's the last administrator." });

            try
            {
                var result = await _accounts.DeleteAsync(user);
                if (!result.Succeeded)
                    return BadRequest(new { Message = "Failed to delete user." });
            }
            catch (DbUpdateException ex)
            {
                _logger.LogWarning(ex, "Deleting user {UserId} blocked by related data", id);
                return Conflict(new { Message = "The user still owns data (lessons, sessions or assignments) and can't be deleted." });
            }

            _logger.LogWarning("User {UserId} deleted by {AdminId}", id, GetCurrentUserId());
            return Ok(new { Message = "User deleted successfully." });
        }

        // IAccountService.GetByIdAsync throws KeyNotFoundException for a missing id.
        private async Task<bloom.Models.Account?> FindAsync(string id)
        {
            try { return await _accounts.GetByIdAsync(id); }
            catch (KeyNotFoundException) { return null; }
        }
    }
}
