using System.ComponentModel.DataAnnotations;
using System.Security.Claims;
using System.Security.Cryptography;
using bloom.Services;
using bloom.Models;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers;

/// <summary>Issued researcher credentials and cookie-based study access.</summary>
[ApiController]
[Route("api/rsr-study/auth")]
public class RsrStudyAuthController : ControllerBase
{
    private readonly UserManager<Account> _users;
    private readonly SignInManager<Account> _signIn;
    public RsrStudyAuthController(UserManager<Account> users, SignInManager<Account> signIn)
    { _users = users; _signIn = signIn; }

    [AllowAnonymous]
    [HttpPost("login")]
    public async Task<IActionResult> Login(StudyLoginRequest request)
    {
        var login = request.Login.Trim();
        var user = await _users.FindByNameAsync(login) ?? await _users.FindByEmailAsync(login);
        if (user == null || !await HasAccess(user))
            return Unauthorized(new { message = "Invalid researcher login or password." });
        var signedIn = await _signIn.PasswordSignInAsync(user, request.Password, isPersistent: false, lockoutOnFailure: true);
        if (!signedIn.Succeeded)
            return Unauthorized(new { message = "Invalid researcher login or password." });
        return Ok(new { user = await Profile(user) });
    }

    [Authorize(Roles = "Researcher,researcher,Admin,admin")]
    [HttpGet("session")]
    public async Task<IActionResult> Session()
    {
        var user = await _users.FindByIdAsync(User.FindFirstValue(ClaimTypes.NameIdentifier) ?? "");
        if (user == null || !await HasAccess(user)) return Forbid();
        return Ok(await Profile(user));
    }

    [AllowAnonymous]
    [HttpPost("logout")]
    public async Task<IActionResult> Logout()
    {
        await _signIn.SignOutAsync();
        return NoContent();
    }

    // Researcher is intentionally not an option on public account registration.
    [Authorize(Roles = "Admin,admin")]
    [HttpPost("researchers")]
    public async Task<IActionResult> CreateResearcher(CreateResearcherRequest request)
    {
        var login = request.Login.Trim();
        if (await _users.FindByNameAsync(login) != null || await _users.FindByEmailAsync(login) != null)
            return Conflict(new { message = "This login is already assigned." });
        var email = request.Email.Trim();
        if (await _users.FindByEmailAsync(email) != null || await _users.FindByNameAsync(email) != null)
            return Conflict(new { message = "This email address is already assigned." });
        var user = new Account { UserName = login, FullName = request.FullName?.Trim(),
            Email = email, Role = "Researcher", CreatedDate = DateTime.UtcNow };
        var temporaryPassword = request.Password ?? ("Aa1!" + Convert.ToBase64String(RandomNumberGenerator.GetBytes(18)));
        var created = await _users.CreateAsync(user, temporaryPassword);
        if (!created.Succeeded) return BadRequest(new { message = string.Join(" ", created.Errors.Select(e => e.Description)) });
        var assigned = await _users.AddToRoleAsync(user, "Researcher");
        if (assigned.Succeeded) assigned = await _users.AddClaimAsync(user, new Claim(StudyAccountProvisioner.PasswordChangeClaim, "true"));
        if (!assigned.Succeeded)
        {
            await _users.DeleteAsync(user);
            return BadRequest(new { message = "Could not assign study access. Please try again." });
        }
        return Ok(new { id = user.Id, login = user.UserName, email = user.Email, role = "Researcher", temporaryPassword });
    }

    [Authorize(Roles = "Researcher,researcher,Admin,admin")]
    [HttpPost("password")]
    public async Task<IActionResult> ChangePassword(StudyPasswordRequest request)
    {
        var user = await _users.GetUserAsync(User);
        if (user == null) return Unauthorized();
        if (request.CurrentPassword == request.NewPassword)
            return BadRequest(new { message = "Choose a different personal password." });
        var changed = await _users.ChangePasswordAsync(user, request.CurrentPassword, request.NewPassword);
        if (!changed.Succeeded)
            return BadRequest(new { message = string.Join(" ", changed.Errors.Select(e => e.Description)) });
        var claims = (await _users.GetClaimsAsync(user)).Where(c => c.Type == StudyAccountProvisioner.PasswordChangeClaim);
        var removed = await _users.RemoveClaimsAsync(user, claims);
        if (!removed.Succeeded) return BadRequest(new { message = "Could not finish password setup. Contact your administrator." });
        await _signIn.RefreshSignInAsync(user);
        return Ok(new { user = await Profile(user) });
    }

    private async Task<bool> HasAccess(Account user) =>
        await _users.IsInRoleAsync(user, "Researcher") || await _users.IsInRoleAsync(user, "Admin");

    private async Task<object> Profile(Account user) => new {
        user.Id, user.UserName, user.Email, user.FullName,
        mustChangePassword = (await _users.GetClaimsAsync(user)).Any(c => c.Type == StudyAccountProvisioner.PasswordChangeClaim),
        role = await _users.IsInRoleAsync(user, "Researcher") ? "Researcher" : "Admin"
    };
}

public class StudyLoginRequest
{
    [Required, StringLength(256)] public string Login { get; set; } = "";
    [Required, StringLength(1024)] public string Password { get; set; } = "";
}
public class StudyPasswordRequest
{
    [Required, StringLength(1024)] public string CurrentPassword { get; set; } = "";
    [Required, StringLength(1024, MinimumLength = 12)] public string NewPassword { get; set; } = "";
}
public class CreateResearcherRequest
{
    [Required, StringLength(256)] public string Login { get; set; } = "";
    [Required, EmailAddress, StringLength(254)] public string Email { get; set; } = "";
    [StringLength(1024)] public string? Password { get; set; }
    [StringLength(256)] public string? FullName { get; set; }
}
