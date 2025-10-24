// bloom
// AccountService.cs
// Class for interfacing with the database, providing useful helper functions.
// Created: 10/22/2025
using System.ComponentModel.DataAnnotations;
using bloom.Models;
using bloom.Models.dto;
using bloom.Data;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc.ApplicationModels;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Internal;
using Microsoft.AspNetCore.Http.HttpResults;

namespace bloom.Services
{
    public class AccountService : IAccountService
    {

        private BloomDbContext _dbContext;
        private UserManager<Account> _userManager;
        private SignInManager<Account> _signInManager;

        public AccountService(BloomDbContext dbContext,
                            UserManager<Account> userManager, 
                            SignInManager<Account> signInManager)
        {
            _dbContext = dbContext;
            _userManager = userManager;
            _signInManager = signInManager;
        }


        public Task<bool> AddToRoleAsync(Account user, string role)
        {

            throw new NotImplementedException();
        }   

        public async Task<IEnumerable<Account?>> GetAllAsync()
        {
            return await _dbContext.Accounts.ToListAsync();
        }

        public Task<Account?> GetByEmailAsync(string email)
        {
            return _dbContext.Accounts.FirstOrDefaultAsync(u => u.Email == email) ?? throw new KeyNotFoundException();
        }

        public Task<Account?> GetByIdAsync(string id)
        {
            return _dbContext.Accounts.FirstOrDefaultAsync(u => u.Id == id) ?? throw new KeyNotFoundException();
        }

        public Task<IList<string>> GetUserRolesAsync(Account user)
        {
            return _userManager.GetRolesAsync(user);
        }

        public Task<bool> IsInRoleAsync(Account user, string role)
        {
            return _userManager.IsInRoleAsync(user, role);
        }

        public Task LogoutAsync()
        {
            return _signInManager.SignOutAsync();
        }

        public async Task<IdentityResult> RegisterAdminAsync(CreateAccountDto user)
        {
            if (user == null)
            {
                throw new ArgumentNullException(nameof(user));
            }

            // todo : do some additinoal validation

            try
            {
                var result = await _userManager.CreateAsync(new Account
                {
                    UserName = user.UserName,
                    Email = user.Email,
                    FullName = user.FullName,
                    EmailConfirmed = false,
                    CreatedDate = DateTime.UtcNow,
                    Role = "Admin"
                }, user.Password);

                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error creating a new admin user", ex);
            }
        }

        public async Task<IdentityResult> RegisterFacilitatorAsync(CreateAccountDto user)
        {
            if (user == null)
            {
                throw new ArgumentNullException(nameof(user));
            }
            // todo : do some additinoal validation

            try
            {
                var result = await _userManager.CreateAsync(new Account
                {
                    UserName = user.UserName,
                    Email = user.Email,
                    FullName = user.FullName,
                    EmailConfirmed = false,
                    CreatedDate = DateTime.UtcNow,
                    Role = "Facilitator"
                }, user.Password);

                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error creating a new facilitator user", ex);
            }
        }

        public async Task<IdentityResult> RegisterStudentAsync(CreateAccountDto user)
        {
            if (user == null)
            {
                throw new ArgumentNullException(nameof(user));
            }
            // todo : do some additinoal validation

            try
            {
                var result = await _userManager.CreateAsync(new Account
                {
                    UserName = user.UserName,
                    Email = user.Email,
                    FullName = user.FullName,
                    EmailConfirmed = false,
                    CreatedDate = DateTime.UtcNow,
                    Role = "Student"
                }, user.Password);

                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error creating a new student user", ex);
            }
        }

        public async Task<SignInResult> SignInAsync(string email, string password)
        {
            if (string.IsNullOrEmpty(email) || string.IsNullOrEmpty(password))
            {
                throw new ArgumentNullException("Email or password is null or empty");
            }

            Account user;

            try
            {
                user = await _dbContext.Accounts.FirstOrDefaultAsync(u => u.Email == email);
            }
            catch (Exception ex)
            {
                throw new KeyNotFoundException("Error retrieving user by email", ex);
            }

            try
            {
                var result = await _signInManager.PasswordSignInAsync(user.UserName ?? "", password, isPersistent: false, lockoutOnFailure: false);
                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error signing in user", ex);
            }

            }
    }
}