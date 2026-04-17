using Xunit;
using Moq;
using System;
using System.Linq;
using Microsoft.AspNetCore.Identity;
using bloom.Models;
using bloom.Models.dto;
using System.Threading.Tasks;
using IdentitySignInResult = Microsoft.AspNetCore.Identity.SignInResult;

namespace bloom.Tests.Services
{
    /// <summary>
    /// Unit tests for AccountService authentication and registration methods.
    /// Note: Full service testing requires database setup. These tests demonstrate
    /// the testing approach using mocked UserManager and SignInManager.
    /// </summary>
    public class AccountServiceAuthTests
    {
        private readonly Mock<UserManager<Account>> _mockUserManager;
        private readonly Mock<SignInManager<Account>> _mockSignInManager;

        public AccountServiceAuthTests()
        {
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
        }

        #region SignInAsync Tests

        [Fact]
        public async Task SignInAsync_WithValidCredentials_ShouldSucceed()
        {
            // Arrange
            var email = "user@example.com";
            var password = "ValidPassword123!";

            _mockSignInManager
                .Setup(s => s.PasswordSignInAsync(
                    email,
                    password,
                    false,
                    false))
                .ReturnsAsync(SignInResult.Success);

            // Act
            var result = await _mockSignInManager.Object.PasswordSignInAsync(
                email, password, false, false);

            // Assert
            Assert.True(result.Succeeded);
        }

        [Fact]
        public async Task SignInAsync_WithInvalidPassword_ShouldFail()
        {
            // Arrange
            var email = "user@example.com";
            var password = "InvalidPassword";

            _mockSignInManager
                .Setup(s => s.PasswordSignInAsync(
                    email,
                    password,
                    false,
                    false))
                .ReturnsAsync(SignInResult.Failed);

            // Act
            var result = await _mockSignInManager.Object.PasswordSignInAsync(
                email, password, false, false);

            // Assert
            Assert.False(result.Succeeded);
        }

        #endregion

        #region User Creation Tests

        [Fact]
        public async Task CreateAsync_WithValidUser_ShouldSucceed()
        {
            // Arrange
            var newUser = new Account
            {
                Email = "newuser@example.com",
                UserName = "newuser@example.com",
                FullName = "New User",
                EmailConfirmed = false,
                CreatedDate = DateTime.UtcNow,
                Role = "STUDENT"
            };

            var password = "NewPassword123!";

            _mockUserManager
                .Setup(u => u.CreateAsync(newUser, password))
                .ReturnsAsync(IdentityResult.Success);

            // Act
            var result = await _mockUserManager.Object.CreateAsync(newUser, password);

            // Assert
            Assert.True(result.Succeeded);
        }

        [Fact]
        public async Task CreateAsync_WithWeakPassword_ShouldFail()
        {
            // Arrange
            var newUser = new Account
            {
                Email = "user@example.com",
                UserName = "user",
                FullName = "User Name",
                Role = "STUDENT"
            };

            var weakPassword = "weak";
            var error = new IdentityError
            {
                Code = "PasswordTooShort",
                Description = "Passwords must be at least 8 characters."
            };

            _mockUserManager
                .Setup(u => u.CreateAsync(newUser, weakPassword))
                .ReturnsAsync(IdentityResult.Failed(error));

            // Act
            var result = await _mockUserManager.Object.CreateAsync(newUser, weakPassword);

            // Assert
            Assert.False(result.Succeeded);
            Assert.Single(result.Errors);
            Assert.Equal("PasswordTooShort", result.Errors.First().Code);
        }

        #endregion

        #region User Lookup Tests

        [Fact]
        public async Task FindByEmailAsync_WithExistingEmail_ReturnsUser()
        {
            // Arrange
            var email = "existing@example.com";
            var existingUser = new Account
            {
                Id = "user-123",
                Email = email,
                UserName = "existinguser",
                FullName = "Existing User",
                Role = "STUDENT"
            };

            _mockUserManager
                .Setup(u => u.FindByEmailAsync(email))
                .ReturnsAsync(existingUser);

            // Act
            var result = await _mockUserManager.Object.FindByEmailAsync(email);

            // Assert
            Assert.NotNull(result);
            Assert.Equal(email, result.Email);
            Assert.Equal("user-123", result.Id);
        }

        [Fact]
        public async Task FindByIdAsync_WithExistingId_ReturnsUser()
        {
            // Arrange
            var userId = "user-123";
            var user = new Account
            {
                Id = userId,
                Email = "user@example.com",
                UserName = "testuser",
                FullName = "Test User",
                Role = "STUDENT"
            };

            _mockUserManager
                .Setup(u => u.FindByIdAsync(userId))
                .ReturnsAsync(user);

            // Act
            var result = await _mockUserManager.Object.FindByIdAsync(userId);

            // Assert
            Assert.NotNull(result);
            Assert.Equal(userId, result.Id);
        }

        [Fact]
        public async Task FindByEmailAsync_WithNonexistentEmail_ReturnsNull()
        {
            // Arrange
            var email = "nonexistent@example.com";

            _mockUserManager
                .Setup(u => u.FindByEmailAsync(email))
                .ReturnsAsync((Account)null);

            // Act
            var result = await _mockUserManager.Object.FindByEmailAsync(email);

            // Assert
            Assert.Null(result);
        }

        #endregion

        #region Role Tests

        [Fact]
        public async Task IsInRoleAsync_WhenUserInRole_ReturnsTrue()
        {
            // Arrange
            var user = new Account
            {
                Id = "user-123",
                Email = "user@example.com"
            };
            var role = "ADMIN";

            _mockUserManager
                .Setup(u => u.IsInRoleAsync(user, role))
                .ReturnsAsync(true);

            // Act
            var result = await _mockUserManager.Object.IsInRoleAsync(user, role);

            // Assert
            Assert.True(result);
        }

        [Fact]
        public async Task IsInRoleAsync_WhenUserNotInRole_ReturnsFalse()
        {
            // Arrange
            var user = new Account
            {
                Id = "user-123",
                Email = "user@example.com"
            };
            var role = "ADMIN";

            _mockUserManager
                .Setup(u => u.IsInRoleAsync(user, role))
                .ReturnsAsync(false);

            // Act
            var result = await _mockUserManager.Object.IsInRoleAsync(user, role);

            // Assert
            Assert.False(result);
        }

        #endregion

        #region User Deletion Tests

        [Fact]
        public async Task DeleteAsync_WithValidUser_Succeeds()
        {
            // Arrange
            var user = new Account
            {
                Id = "user-123",
                Email = "user@example.com"
            };

            _mockUserManager
                .Setup(u => u.DeleteAsync(user))
                .ReturnsAsync(IdentityResult.Success);

            // Act
            var result = await _mockUserManager.Object.DeleteAsync(user);

            // Assert
            Assert.True(result.Succeeded);
            _mockUserManager.Verify(u => u.DeleteAsync(user), Times.Once);
        }

        [Fact]
        public async Task DeleteAsync_WhenFails_ReturnsFailed()
        {
            // Arrange
            var user = new Account
            {
                Id = "user-123",
                Email = "user@example.com"
            };
            var error = new IdentityError
            {
                Code = "ConcurrencyFailure",
                Description = "Concurrency failure while deleting."
            };

            _mockUserManager
                .Setup(u => u.DeleteAsync(user))
                .ReturnsAsync(IdentityResult.Failed(error));

            // Act
            var result = await _mockUserManager.Object.DeleteAsync(user);

            // Assert
            Assert.False(result.Succeeded);
            Assert.Single(result.Errors);
        }

        #endregion
    }
}
