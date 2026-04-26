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
using System.Security.Cryptography;

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

        public async Task<IdentityResult> DeleteAsync(Account user)
        {
            return await _userManager.DeleteAsync(user);
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

        public async Task<(IdentityResult Result, string GeneratedPassword)> RegisterStudentWithGeneratedPasswordAsync(CreateStudentByPrivilegedUserDto dto)
        {
            if (dto == null)
            {
                throw new ArgumentNullException(nameof(dto));
            }

            var password = GenerateTemporaryPassword();

            try
            {
                var result = await _userManager.CreateAsync(new Account
                {
                    Email = dto.Email,
                    FullName = dto.FullName,
                    EmailConfirmed = false,
                    CreatedDate = DateTime.UtcNow,
                    Role = "Student",
                    UserName = dto.Email
                }, password);

                return (result, result.Succeeded ? password : string.Empty);
            }
            catch (Exception ex)
            {
                throw new Exception("Error creating student account", ex);
            }
        }

        private static string GenerateTemporaryPassword()
        {
            const string upper = "ABCDEFGHJKLMNPQRSTUVWXYZ";
            const string lower = "abcdefghjkmnpqrstuvwxyz";
            const string digits = "23456789";
            const string all = upper + lower + digits;

            var chars = new char[8];
            var bytes = new byte[8];
            RandomNumberGenerator.Fill(bytes);

            // Guarantee at least one char from each required class
            chars[0] = upper[bytes[0] % upper.Length];
            chars[1] = lower[bytes[1] % lower.Length];
            chars[2] = digits[bytes[2] % digits.Length];

            // Fill remaining positions from the full set
            for (int i = 3; i < 8; i++)
            {
                chars[i] = all[bytes[i] % all.Length];
            }

            // Shuffle using Fisher-Yates with a fresh random buffer
            var shuffleBytes = new byte[8];
            RandomNumberGenerator.Fill(shuffleBytes);
            for (int i = 7; i > 0; i--)
            {
                int j = shuffleBytes[i] % (i + 1);
                (chars[i], chars[j]) = (chars[j], chars[i]);
            }

            return new string(chars);
        }

        public async Task<IdentityResult> RegisterTeacherAsync(CreateAccountDto user)
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
                    Role = "Teacher"
                }, user.Password);

                return result;
            }
            catch (Exception ex)
            {
                throw new Exception("Error creating a new teacher user", ex);
            }
        }

        public async Task<IdentityResult> RegisterSLPAsync(CreateAccountDto user)
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

            user = await _dbContext.Accounts.FirstOrDefaultAsync(u => u.Email == email);
            if (user == null)
            {
                return Microsoft.AspNetCore.Identity.SignInResult.Failed;
            }

            return await _signInManager.PasswordSignInAsync(user.UserName ?? "", password, isPersistent: false, lockoutOnFailure: false);

            }
    }
}