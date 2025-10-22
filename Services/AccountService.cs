// bloom
// AccountService.cs
// Class for interfacing with the database, providing useful helper functions.
// Created: 10/22/2025
using bloom.Models;
using Microsoft.AspNetCore.Identity;

namespace bloom.Services
{
    public class AccountService : IAccountService
    {
        public Task<bool> AddToRoleAsync(Account user, string role)
        {
            throw new NotImplementedException();
        }

        public Task<IEnumerable<Account?>> GetAllAsync()
        {
            throw new NotImplementedException();
        }

        public Task<Account?> GetByEmailAsync(string email)
        {
            throw new NotImplementedException();
        }

        public Task<Account?> GetByIdAsync(string id)
        {
            throw new NotImplementedException();
        }

        public Task<IList<string>> GetUserRolesAsync(Account user)
        {
            throw new NotImplementedException();
        }

        public Task<bool> IsInRoleAsync(Account user, string role)
        {
            throw new NotImplementedException();
        }

        public Task LogoutAsync()
        {
            throw new NotImplementedException();
        }

        public Task<IdentityResult> RegisterAdminAsync(AdminUser user, string password)
        {
            throw new NotImplementedException();
        }

        public Task<IdentityResult> RegisterFacilitatorAsync(FacilitatorUser user, string password)
        {
            throw new NotImplementedException();
        }

        public Task<IdentityResult> RegisterStudentAsync(StudentUser user, string password)
        {
            throw new NotImplementedException();
        }

        public Task<SignInResult> SignInAsync(string email, string password)
        {
            throw new NotImplementedException();
        }
    }
}