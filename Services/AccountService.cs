// bloom
// AccountService.cs
// Class for interfacing with the database, providing useful helper functions.
// Created: 10/22/2025
using System.ComponentModel.DataAnnotations;
using bloom.Models;
using bloom.Models.dto;
using Bloom.Data;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc.ApplicationModels;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Internal;

namespace bloom.Services
{
    public class AccountService : IAccountService
    {

        private IDbContextFactory<BloomDbContext> _dbContextFactory;
        private UserManager<Account> _userManager;
        private SignInManager<Account> _signInManager;

        public AccountService(DbContextFactory<BloomDbContext> dbContextFactory, UserManager<Account> userManager, SignInManager<Account> signInManager)
        {
            _dbContextFactory = dbContextFactory;
            _userManager = userManager;
            _signInManager = signInManager;
        }


        public Task<bool> AddToRoleAsync(Account user, string role)
        {
            BloomDbContext context = _dbContextFactory.CreateDbContext();

            throw new NotImplementedException();
        }   

        public async Task<IEnumerable<Account?>> GetAllAsync()
        {
            BloomDbContext context = _dbContextFactory.CreateDbContext();
            return await context.Accounts.ToListAsync();
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

        public Task<SignInResult> SignInAsync(string email, string password)
        {
            if (string.IsNullOrEmpty(email) || string.IsNullOrEmpty(password))
            {
                throw new ArgumentNullException("Email or password is null or empty");
            }

            try
            {
                var result = _signInManager.PasswordSignInAsync(email, password, isPersistent: false, lockoutOnFailure: false);
                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error signing in user", ex);
            }
        
        }
    }
}