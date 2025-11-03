using System.ComponentModel.DataAnnotations;
using bloom.Services;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Identity.Data;
using Microsoft.AspNetCore.Mvc;
using System.Security.Claims;
using Microsoft.AspNetCore.Authentication.Cookies;
using Microsoft.AspNetCore.Authentication;
using Microsoft.EntityFrameworkCore.Query.SqlExpressions;
using Microsoft.EntityFrameworkCore.Metadata.Internal;

namespace bloom.Controllers;

[ApiController]
[Route("[controller]")]
public class AccountController : ControllerBase
{

    private readonly IAccountService _accountService;

    public AccountController(IAccountService accountService)
    {
        _accountService = accountService;
    }

    [HttpPost]
    [Route("login")]
    public async Task<IActionResult> Login([FromBody] LoginDto account)
    {
        if (!ModelState.IsValid)
        {
            return BadRequest(ModelState);
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
                EmailConfirmed = user.EmailConfirmed
            }
        });

    }

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
            case "FACILITATOR":
                result = await _accountService.RegisterFacilitatorAsync(account);
                break;
            case "STUDENT":
                result = await _accountService.RegisterStudentAsync(account);
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
                return BadRequest();
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
                    EmailConfirmed = new_user.EmailConfirmed
                }
            });
        }
        else
        {
            Console.WriteLine("Step D: Creation failed with errors:");
            foreach (var error in result.Errors)
            {
                Console.WriteLine($"Error: {error.Description}");
            }
        }
        return BadRequest();
    }

    

}