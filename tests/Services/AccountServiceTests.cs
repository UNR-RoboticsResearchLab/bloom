using Xunit;
using Moq;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Identity;
using Microsoft.Data.Sqlite;
using Microsoft.EntityFrameworkCore;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;

namespace bloom.Tests.Services
{
    /// <summary>
    /// Unit tests for AccountService. Exercises the real AccountService instance (constructed
    /// with mocked UserManager/SignInManager and a real SQLite in-memory BloomDbContext, since
    /// SignInAsync/GetByEmailAsync/GetByIdAsync/DeleteAsync all query the DbContext directly
    /// rather than going through UserManager) -- earlier versions of this file called the
    /// mocked UserManager/SignInManager methods directly, which exercised Moq's own behavior
    /// rather than any logic in AccountService itself.
    /// </summary>
    public class AccountServiceTests : IDisposable
    {
        private readonly SqliteConnection _connection;
        private readonly BloomDbContext _db;
        private readonly Mock<UserManager<Account>> _mockUserManager;
        private readonly Mock<SignInManager<Account>> _mockSignInManager;
        private readonly AccountService _accountService;

        public AccountServiceTests()
        {
            _connection = new SqliteConnection("DataSource=:memory:");
            _connection.Open();
            var options = new DbContextOptionsBuilder<BloomDbContext>().UseSqlite(_connection).Options;
            _db = new BloomDbContext(options);
            _db.Database.EnsureCreated();

            var userStore = new Mock<IUserStore<Account>>();
            _mockUserManager = new Mock<UserManager<Account>>(
                userStore.Object, null!, null!, null!, null!, null!, null!, null!, null!);

            var contextAccessor = new Mock<Microsoft.AspNetCore.Http.IHttpContextAccessor>();
            var userPrincipalFactory = new Mock<IUserClaimsPrincipalFactory<Account>>();
            _mockSignInManager = new Mock<SignInManager<Account>>(
                _mockUserManager.Object,
                contextAccessor.Object,
                userPrincipalFactory.Object,
                null!, null!, null!, null!);

            _accountService = new AccountService(_db, _mockUserManager.Object, _mockSignInManager.Object);
        }

        public void Dispose()
        {
            _db.Dispose();
            _connection.Dispose();
        }

        private Account SeedAccount(string email, string role = "Student")
        {
            var account = new Account
            {
                Email = email,
                UserName = email,
                FullName = "Test Account",
                Role = role,
                CreatedDate = DateTime.UtcNow
            };
            _db.Accounts.Add(account);
            _db.SaveChanges();
            return account;
        }

        private static CreateAccountDto NewCreateAccountDto(string email = "newuser@example.com") => new()
        {
            FullName = "New User",
            Email = email,
            Password = "NewPassword123!",
            SelectedRole = "Student",
            UserName = email
        };

        #region SignInAsync

        [Fact]
        public async Task SignInAsync_WithValidCredentials_Succeeds()
        {
            var account = SeedAccount("user@example.com");
            _mockSignInManager
                .Setup(s => s.PasswordSignInAsync(account.UserName!, "ValidPassword123!", false, false))
                .ReturnsAsync(SignInResult.Success);

            var result = await _accountService.SignInAsync("user@example.com", "ValidPassword123!");

            Assert.True(result.Succeeded);
        }

        [Fact]
        public async Task SignInAsync_WithInvalidPassword_Fails()
        {
            var account = SeedAccount("user2@example.com");
            _mockSignInManager
                .Setup(s => s.PasswordSignInAsync(account.UserName!, "WrongPassword", false, false))
                .ReturnsAsync(SignInResult.Failed);

            var result = await _accountService.SignInAsync("user2@example.com", "WrongPassword");

            Assert.False(result.Succeeded);
        }

        [Fact]
        public async Task SignInAsync_UnknownEmail_ReturnsFailedWithoutCallingSignInManager()
        {
            var result = await _accountService.SignInAsync("nobody@example.com", "whatever");

            Assert.False(result.Succeeded);
            _mockSignInManager.Verify(
                s => s.PasswordSignInAsync(It.IsAny<string>(), It.IsAny<string>(), It.IsAny<bool>(), It.IsAny<bool>()),
                Times.Never);
        }

        #endregion

        #region RegisterStudentAsync / RegisterWithRoleAsync

        [Fact]
        public async Task RegisterStudentAsync_WithValidUser_SucceedsAndAddsStudentRole()
        {
            _mockUserManager
                .Setup(u => u.CreateAsync(It.IsAny<Account>(), "NewPassword123!"))
                .ReturnsAsync(IdentityResult.Success);
            _mockUserManager
                .Setup(u => u.AddToRoleAsync(It.IsAny<Account>(), "Student"))
                .ReturnsAsync(IdentityResult.Success);

            var result = await _accountService.RegisterStudentAsync(NewCreateAccountDto());

            Assert.True(result.Succeeded);
            _mockUserManager.Verify(u => u.AddToRoleAsync(It.IsAny<Account>(), "Student"), Times.Once);
        }

        [Fact]
        public async Task RegisterStudentAsync_WithWeakPassword_FailsAndDoesNotAddRole()
        {
            var error = new IdentityError { Code = "PasswordTooShort", Description = "Too short." };
            _mockUserManager
                .Setup(u => u.CreateAsync(It.IsAny<Account>(), It.IsAny<string>()))
                .ReturnsAsync(IdentityResult.Failed(error));

            var result = await _accountService.RegisterStudentAsync(NewCreateAccountDto());

            Assert.False(result.Succeeded);
            Assert.Equal("PasswordTooShort", result.Errors.First().Code);
            _mockUserManager.Verify(u => u.AddToRoleAsync(It.IsAny<Account>(), It.IsAny<string>()), Times.Never);
        }

        [Fact]
        public async Task RegisterStudentAsync_BuildsAccountWithStudentRoleAndUnconfirmedEmail()
        {
            // The one piece of real logic in RegisterWithRoleAsync: it builds the Account object
            // itself (role, EmailConfirmed=false, etc.) before delegating to UserManager.
            Account? createdAccount = null;
            _mockUserManager
                .Setup(u => u.CreateAsync(It.IsAny<Account>(), It.IsAny<string>()))
                .Callback<Account, string>((a, _) => createdAccount = a)
                .ReturnsAsync(IdentityResult.Success);
            _mockUserManager
                .Setup(u => u.AddToRoleAsync(It.IsAny<Account>(), It.IsAny<string>()))
                .ReturnsAsync(IdentityResult.Success);

            var dto = NewCreateAccountDto("student-role-check@example.com");
            await _accountService.RegisterStudentAsync(dto);

            Assert.NotNull(createdAccount);
            Assert.Equal("Student", createdAccount!.Role);
            Assert.False(createdAccount.EmailConfirmed);
            Assert.Equal(dto.Email, createdAccount.Email);
            Assert.Equal(dto.UserName, createdAccount.UserName);
            Assert.Equal(dto.FullName, createdAccount.FullName);
        }

        #endregion

        #region GetByEmailAsync / GetByIdAsync

        [Fact]
        public async Task GetByEmailAsync_WithExistingEmail_ReturnsAccount()
        {
            var seeded = SeedAccount("existing@example.com");

            var result = await _accountService.GetByEmailAsync("existing@example.com");

            Assert.NotNull(result);
            Assert.Equal(seeded.Id, result!.Id);
        }

        [Fact]
        public async Task GetByEmailAsync_WithNonexistentEmail_ThrowsKeyNotFoundException()
        {
            await Assert.ThrowsAsync<KeyNotFoundException>(
                () => _accountService.GetByEmailAsync("nonexistent@example.com"));
        }

        [Fact]
        public async Task GetByIdAsync_WithExistingId_ReturnsAccount()
        {
            var seeded = SeedAccount("byid@example.com");

            var result = await _accountService.GetByIdAsync(seeded.Id);

            Assert.NotNull(result);
            Assert.Equal("byid@example.com", result!.Email);
        }

        #endregion

        #region IsInRoleAsync

        [Fact]
        public async Task IsInRoleAsync_WhenUserInRole_ReturnsTrue()
        {
            var account = SeedAccount("admin@example.com", role: "Admin");
            _mockUserManager.Setup(u => u.IsInRoleAsync(account, "Admin")).ReturnsAsync(true);

            Assert.True(await _accountService.IsInRoleAsync(account, "Admin"));
        }

        [Fact]
        public async Task IsInRoleAsync_WhenUserNotInRole_ReturnsFalse()
        {
            var account = SeedAccount("nonadmin@example.com", role: "Student");
            _mockUserManager.Setup(u => u.IsInRoleAsync(account, "Admin")).ReturnsAsync(false);

            Assert.False(await _accountService.IsInRoleAsync(account, "Admin"));
        }

        #endregion

        #region DeleteAsync

        [Fact]
        public async Task DeleteAsync_WithNoDependents_DelegatesToUserManager()
        {
            var account = SeedAccount("delete-me@example.com");
            _mockUserManager.Setup(u => u.DeleteAsync(account)).ReturnsAsync(IdentityResult.Success);

            var result = await _accountService.DeleteAsync(account);

            Assert.True(result.Succeeded);
            _mockUserManager.Verify(u => u.DeleteAsync(account), Times.Once);
        }

        [Fact]
        public async Task DeleteAsync_RemovesDependentSlpClientRelationshipsFirst()
        {
            // The one piece of real logic in DeleteAsync: before delegating to UserManager, it
            // cleans up any SLPClient rows that reference this account as either the SLP or the
            // student side of the relationship, so deleting an account never leaves a dangling FK.
            var slp = SeedAccount("slp@example.com", role: "SLP");
            var student = SeedAccount("student@example.com", role: "Student");
            _db.SLPClients.Add(new SLPClient { Name = "link", SlpId = slp.Id, StudentId = student.Id });
            _db.SaveChanges();
            _mockUserManager.Setup(u => u.DeleteAsync(slp)).ReturnsAsync(IdentityResult.Success);

            var result = await _accountService.DeleteAsync(slp);

            Assert.True(result.Succeeded);
            Assert.Empty(_db.SLPClients.Where(c => c.SlpId == slp.Id));
        }

        [Fact]
        public async Task DeleteAsync_WhenUserManagerFails_ReturnsFailed()
        {
            var account = SeedAccount("delete-fail@example.com");
            var error = new IdentityError { Code = "ConcurrencyFailure", Description = "Concurrency failure." };
            _mockUserManager.Setup(u => u.DeleteAsync(account)).ReturnsAsync(IdentityResult.Failed(error));

            var result = await _accountService.DeleteAsync(account);

            Assert.False(result.Succeeded);
            Assert.Single(result.Errors);
        }

        #endregion
    }
}
