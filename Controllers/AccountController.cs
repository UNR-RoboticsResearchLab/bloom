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
    public async Task<IActionResult> Login(LoginDto account)
    {
        var res = await _accountService.SignInAsync(account.Email, account.Password);

        if (!res.Succeeded)
        {
            return Unauthorized(new { Message = "Invalid login attempt." });
        }

        var user = await _accountService.GetByEmailAsync(account.Email);
        if (user == null)
        {
            return NotFound(new { Message = "User not found. " });
        }

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
                FullName = user.FullName
            }
        });

    }

    [HttpPost]
    public async Task<IActionResult> Create(CreateAccountDto account)
    {
        return NotFound();
    }

    

}