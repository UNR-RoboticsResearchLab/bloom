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
        Task<IdentityResult> RegisterFacilitatorAsync(CreateAccountDto user);

        // this is kind of idiosyncratic since they wont register themselves ? 
        Task<IdentityResult> RegisterStudentAsync(CreateAccountDto user);
        Task<IdentityResult> RegisterAdminAsync(Account user, string password);
        Task<IdentityResult> RegisterFacilitatorAsync(Account user, string password);        
        Task<IdentityResult> RegisterStudentAsync(Account user, string password);

        // Auth Accounts
        Task<SignInResult> SignInAsync(string email, string password);
        Task LogoutAsync();

        // Roles and Claims
        Task<bool> AddToRoleAsync(Account user, string role);
        Task<bool> IsInRoleAsync(Account user, string role);
        Task<IList<string>> GetUserRolesAsync(Account user);
    }   
    
}