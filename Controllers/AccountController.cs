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
    public async Task<IActionResult> Create(CreateAccountDto account)
    {
        var result = new IdentityResult();
        if (account.SelectedRole == "")
        {
            result = await _accountService.RegisterAdminAsync(account);
        }
        else if (account.SelectedRole == "")
        {
            result = await _accountService.RegisterFacilitatorAsync(account);
        }
        else
        {
            result = await _accountService.RegisterStudentAsync(account);
        }

        if (result.Succeeded)
        {
            var new_user = await _accountService.GetByEmailAsync(account.Email);

            if (new_user == null)
            {
                return BadRequest();
            }

            return Ok( new
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
        return BadRequest();
    }

    

}