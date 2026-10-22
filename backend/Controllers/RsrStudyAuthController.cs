using System.ComponentModel.DataAnnotations;
using System.Security.Claims;
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
        var user = await _users.FindByNameAsync(request.Login.Trim());
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
        if (await _users.FindByNameAsync(login) != null)
            return Conflict(new { message = "This login is already assigned." });
        var user = new Account { UserName = login, FullName = request.FullName?.Trim(),
            Role = "Researcher", CreatedDate = DateTime.UtcNow };
        var created = await _users.CreateAsync(user, request.Password);
        if (!created.Succeeded) return BadRequest(new { message = string.Join(" ", created.Errors.Select(e => e.Description)) });
        var assigned = await _users.AddToRoleAsync(user, "Researcher");
        if (!assigned.Succeeded)
        {
            await _users.DeleteAsync(user);
            return BadRequest(new { message = "Could not assign study access. Please try again." });
        }
        return Ok(new { id = user.Id, login = user.UserName, role = "Researcher" });
    }

    private async Task<bool> HasAccess(Account user) =>
        await _users.IsInRoleAsync(user, "Researcher") || await _users.IsInRoleAsync(user, "Admin");

    private async Task<object> Profile(Account user) => new {
        user.Id, user.UserName, user.Email, user.FullName,
        role = await _users.IsInRoleAsync(user, "Researcher") ? "Researcher" : "Admin"
    };
}

public class StudyLoginRequest
{
    [Required, StringLength(256)] public string Login { get; set; } = "";
    [Required, StringLength(1024)] public string Password { get; set; } = "";
}
public class CreateResearcherRequest : StudyLoginRequest
{
    [StringLength(256)] public string? FullName { get; set; }
}
