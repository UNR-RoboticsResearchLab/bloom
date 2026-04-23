using System.ComponentModel.DataAnnotations;
using bloom.Services;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Identity.Data;
using Microsoft.AspNetCore.Mvc;
using System.Security.Claims;
using Microsoft.AspNetCore.Authentication.Cookies;
using Microsoft.AspNetCore.Authentication;
using Microsoft.EntityFrameworkCore.Query.SqlExpressions;
using Microsoft.EntityFrameworkCore.Metadata.Internal;

namespace bloom.Controllers;

/// <summary>
/// Handles user authentication and account management.
/// Supports cookie-based login, role-based registration (Admin, Student, SLP),
/// profile retrieval by ID, and account deletion.
/// </summary>
[ApiController]
[Route("api/[controller]")]
public class AccountController : ControllerBase
{

    private readonly IAccountService _accountService;
    private readonly ISLPClientService _slpClientService;

    public AccountController(IAccountService accountService, ISLPClientService slpClientService)
    {
        _accountService = accountService;
        _slpClientService = slpClientService;
    }

    [HttpPost]
    [Route("login")]
    public async Task<IActionResult> Login([FromBody] LoginDto account)
    {
        if (!ModelState.IsValid)
        {
            return BadRequest(ModelState);
        }

        if (account.Email == null || account.Password == null)
        {
            return BadRequest(new { Message = "Email and Password are required." });
        }

        var res = await _accountService.SignInAsync(account.Email, account.Password);

        if (!res.Succeeded)
        {
            return BadRequest(new { Message = "Invalid login attempt." });
        }

        var user = await _accountService.GetByEmailAsync(account.Email);
        if (user == null)
        {
            return BadRequest(new { Message = "User not found. " });
        }

        // todo: move to helper function 

        var claims = new List<Claim>
        {
            new Claim(ClaimTypes.NameIdentifier, user.Id.ToString())
        };

        var identity = new ClaimsIdentity(claims, CookieAuthenticationDefaults.AuthenticationScheme);
        var principal = new ClaimsPrincipal(identity);

        await HttpContext.SignInAsync(CookieAuthenticationDefaults.AuthenticationScheme, principal);

        return Ok(new
        {
            Message = "Login successful",
            User = new
            {
                Id = user.Id,
                UserName = user.UserName,
                Email = user.Email,
                FullName = user.FullName,
                EmailConfirmed = user.EmailConfirmed,
                Role = user.Role
            }
        });

    }

    [Authorize(Policy = "CanCreateAccount")]
    [HttpPost]
    [Route("create")]
    public async Task<IActionResult> Create([FromBody] CreateAccountDto account)
    {
        Console.WriteLine($"Step A: Starting account creation {account?.Email}, Role = {account?.SelectedRole}");

        // badrequest malformed requests
        if (!ModelState.IsValid)
        {
            return BadRequest(ModelState);
        }

        var result = new IdentityResult();

        Console.WriteLine($"Step B: Role = {account?.SelectedRole}");
        switch (account?.SelectedRole.ToUpper())
        {
            case "ADMIN":
                result = await _accountService.RegisterAdminAsync(account);
                break;
            case "STUDENT":
                result = await _accountService.RegisterStudentAsync(account);
                break;
            case "SLP":
                result = await _accountService.RegisterSLPAsync(account);
                break;
            default:
                return BadRequest(new { Message = "Invalid role specified." });
        }

        Console.WriteLine($"Step C: Result succeeded? {result.Succeeded}");
        if (result.Succeeded)
        {
            var new_user = await _accountService.GetByEmailAsync(account.Email);

            if (new_user == null)
            {
                return BadRequest("An error occured during account creation.");
            }

            return Ok(new
            {
                Message = "Creation Success",
                User = new
                {
                    Id = new_user.Id,
                    FullName = new_user.FullName,
                    UserName = new_user.UserName,
                    Email = new_user.Email,
                    EmailConfirmed = new_user.EmailConfirmed,
                    Role = new_user.Role
                }
            });
        }
        else
        {
            Console.WriteLine("Step D: Creation failed with errors:");
            foreach (var error in result.Errors)
            {
                Console.WriteLine($"Error: {error.Description}");
                return BadRequest(new { Message = error.Description });
            }
        }
        return BadRequest();
    }

    [Authorize(Policy = "CanCreateAccount")]
    [HttpPost]
    [Route("create/student")]
    public async Task<IActionResult> CreateStudentAccount([FromBody] CreateStudentByPrivilegedUserDto dto)
    {
        if (!ModelState.IsValid)
        {
            return BadRequest(ModelState);
        }

        var (result, generatedPassword) = await _accountService.RegisterStudentWithGeneratedPasswordAsync(dto);

        if (result.Succeeded)
        {
            var newUser = await _accountService.GetByEmailAsync(dto.Email);
            if (newUser == null)
            {
                return BadRequest(new { Message = "An error occurred during account creation." });
            }

            var slpId = User.FindFirstValue(ClaimTypes.NameIdentifier);
            if (slpId != null)
            {
                await _slpClientService.CreateAsync(newUser.FullName ?? newUser.Email ?? newUser.Id, slpId, newUser.Id);
            }

            return Ok(new
            {
                Message = "Student account created successfully.",
                User = new
                {
                    Id = newUser.Id,
                    FullName = newUser.FullName,
                    Email = newUser.Email,
                    Role = newUser.Role
                },
                TemporaryPassword = generatedPassword
            });
        }

        foreach (var error in result.Errors)
        {
            return BadRequest(new { Message = error.Description });
        }

        return BadRequest();
    }

    [Authorize]
    [HttpDelete]
    [Route("{id}")]
    public async Task<IActionResult> DeleteUserProfile(string id)
    {
        var user = await _accountService.GetByIdAsync(id);

        if (user == null)
        {
            return BadRequest("User not found");
        }

        var result = await _accountService.DeleteUserAsync(id);

        if (!result.Succeeded)
        {
            return BadRequest(new { Message = result.Errors});
        }
        
        return Ok(user);

        
    }

    // TODO slp make student account


    [Authorize]
    [HttpGet]
    [Route("{id}")]
    public async Task<IActionResult> GetUserProfile(string id)
    {
        var user = await _accountService.GetByIdAsync(id);
        if (user == null)
        {
            return BadRequest(new { Message = "User not found." });
        }

        var newUser = new
        {
            Id = user.Id,
            FullName = user.FullName,
            UserName = user.UserName,
            Email = user.Email,
            EmailConfirmed = user.EmailConfirmed,
            Role = user.Role,
            RegisteredRobots = user.RegisteredRobots,
            AssignedAssignments = user.AssignedAssignments,
            Lessons = user.CreatedLessons

        };

        return Ok(newUser);
    }
}