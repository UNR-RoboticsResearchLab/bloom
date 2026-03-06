// bloom
// AccountService.cs
// Class for interfacing with the database, providing useful helper functions.
// Created: 10/22/2025
using bloom.Models;
using bloom.Models.dto;
using bloom.Data;
using Microsoft.AspNetCore.Identity;
using System.ComponentModel.DataAnnotations;
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


        public async Task<Account?> GetByEmailAsync(string email)
        {
            return await _dbContext.Accounts.FirstOrDefaultAsync(u => u.Email == email) ?? throw new KeyNotFoundException();
        }

        public async Task<Account?> GetByIdAsync(string id)
        {
            return await _dbContext.Accounts.FirstOrDefaultAsync(u => u.Id.ToString() == id.ToString()) ?? throw new KeyNotFoundException();
        }

        public async Task<IList<string>> GetUserRolesByIdAsync(string id)
        {
            var user = _userManager.Users.FirstOrDefault(u => u.Id.ToString() == id.ToString());
            if (user == null)
            {
                throw new KeyNotFoundException("User not found");
            }
            
            return await _userManager.GetRolesAsync(user);
        }

        public async Task<bool> IsInRoleAsync(Account user, string role)
        {
            return await _userManager.IsInRoleAsync(user, role);
        }

        public async Task LogoutAsync()
        {
            await _signInManager.SignOutAsync();
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

        public async Task<IdentityResult> RegisterSLPAsync(CreateAccountDto user)
        {
            if (user == null)
            {
                throw new ArgumentNullException(nameof(user));
            }
            // todo : do some additional validation

            try
            {
                var result = await _userManager.CreateAsync(new Account
                {
                    Email = user.Email,
                    FullName = user.FullName,
                    EmailConfirmed = false,
                    CreatedDate = DateTime.UtcNow,
                    Role = "SLP"
                }, user.Password);

                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error creating a new SLP user", ex);
            }
        }

        public async Task<SignInResult> SignInAsync(string email, string password)
        {
            if (string.IsNullOrEmpty(email) || string.IsNullOrEmpty(password))
            {
                throw new ArgumentNullException("Email or password is null or empty");
            }

            Account? user;

            try
            {
                user = await _dbContext.Accounts.FirstOrDefaultAsync(u => u.Email == email);
                if (user == null)
                {
                    throw new KeyNotFoundException("User not found");
                }
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

        public async Task<IdentityResult> DeleteUserAsync(string accountId)
        {
            try {

                var userRecord = await _dbContext.Accounts.Where(a => a.Id == accountId).ToListAsync();


                if (userRecord == null || userRecord.Count == 0)
                {
                    return IdentityResult.Failed(
                        new IdentityError[]{
                            new IdentityError {
                                Code = "404",
                                Description = "User record not found"
                            }
                        }
                    );
                }

                _dbContext.Accounts.RemoveRange(userRecord);

                var result = await _dbContext.SaveChangesAsync();
                
                if (result == 0)
                {
                    return IdentityResult.Failed(
                        new IdentityError[]{
                            new IdentityError {
                                Code = "300",
                                Description = "Internal error deleting user record"
                            }
                        }
                    );
                }

                return IdentityResult.Success;
                
            }
            catch (Exception ex)
            {
                return IdentityResult.Failed(new IdentityError[]
                {
                    new IdentityError
                    {
                        Code = "500",
                        Description = ex.Message
                    }
                });
            }

            
        }
    }
}