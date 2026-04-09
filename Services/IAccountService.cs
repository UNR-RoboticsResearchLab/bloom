// bloom
// IAccountService.cs
// Interface defining behavior of AccountService.cs (WIP)
// Created: 10/22/2025

using bloom.Models;
using bloom.Models.dto;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.SignalR;

namespace bloom.Services
{
    public interface IAccountService
    {

        // Get Accounts
        Task<Account?> GetByIdAsync(string id);
        Task<Account?> GetByEmailAsync(string email);
        Task<IEnumerable<Account?>> GetAllAsync();

        // Register Accounts
        Task<IdentityResult> RegisterAdminAsync(CreateAccountDto user);

        // Delete Account
        Task<IdentityResult> DeleteUserAsync(string id);

        // this is kind of idiosyncratic since they wont register themselves ?
        Task<IdentityResult> RegisterStudentAsync(CreateAccountDto user);

        // Used by privileged users (Admin/SLP) to create a student account with a server-generated starting password
        Task<(IdentityResult Result, string GeneratedPassword)> RegisterStudentWithGeneratedPasswordAsync(CreateStudentByPrivilegedUserDto dto);

        Task<IdentityResult> RegisterSLPAsync(CreateAccountDto user);
    

        // Auth Accounts
        Task<SignInResult> SignInAsync(string email, string password);
        Task LogoutAsync();

        // Roles and Claims
        Task<bool> AddToRoleAsync(Account user, string role);
        Task<bool> IsInRoleAsync(Account user, string role);
        Task<IList<string>> GetUserRolesByIdAsync(string userId);
    }   
    
}