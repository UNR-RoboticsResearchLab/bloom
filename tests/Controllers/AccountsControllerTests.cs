using Xunit;
using Moq;
using System.Linq;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Authentication;
using Microsoft.AspNetCore.Http;
using System.Security.Claims;
using bloom.Controllers;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using System.Threading.Tasks;
using IdentitySignInResult = Microsoft.AspNetCore.Identity.SignInResult;


namespace bloom.Tests.Controllers
{
    public class AccountsControllerTests
    {
        private readonly Mock<IAccountService> _mockAccountService;
        private readonly AccountsController _controller;
        private readonly Mock<HttpContext> _mockHttpContext;
        private readonly Mock<IAuthenticationService> _mockAuthService;

        public AccountsControllerTests()
        {
            _mockAccountService = new Mock<IAccountService>();
            _mockHttpContext = new Mock<HttpContext>();
            _mockAuthService = new Mock<IAuthenticationService>();

            _controller = new AccountsController(_mockAccountService.Object)
            {
                ControllerContext = new ControllerContext
                {
                    HttpContext = _mockHttpContext.Object
                }
            };
        }

        #region Login Tests

        [Fact]
        public async Task Login_WithValidCredentials_ReturnsOkWithUserData()
        {
            // Arrange
            var loginDto = new LoginDto
            {
                Email = "test@example.com",
                Password = "Password123!"
            };

            var user = new Account
            {
                Id = "user-123",
                Email = "test@example.com",
                UserName = "testuser",
                FullName = "Test User",
                EmailConfirmed = true,
                Role = "STUDENT"
            };

            _mockAccountService
                .Setup(s => s.SignInAsync(loginDto.Email, loginDto.Password))
                .ReturnsAsync(IdentitySignInResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(loginDto.Email))
                .ReturnsAsync(user);

            // Act
            var result = await _controller.Login(loginDto);

            // Assert
            var okResult = Assert.IsType<OkObjectResult>(result);
            Assert.Equal(200, okResult.StatusCode);

            var responseValue = okResult.Value;
            Assert.NotNull(responseValue);

            var responseProps = responseValue.GetType().GetProperties();
            var messageProperty = responseProps.FirstOrDefault(p => p.Name == "Message");
            Assert.NotNull(messageProperty);
            Assert.Equal("Login successful", messageProperty.GetValue(responseValue));
        }

        [Fact]
        public async Task Login_WithInvalidCredentials_ReturnsBadRequest()
        {
            // Arrange
            var loginDto = new LoginDto
            {
                Email = "test@example.com",
                Password = "WrongPassword"
            };

            _mockAccountService
                .Setup(s => s.SignInAsync(loginDto.Email, loginDto.Password))
                .ReturnsAsync(IdentitySignInResult.Failed);

            // Act
            var result = await _controller.Login(loginDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        [Fact]
        public async Task Login_WithNullEmail_ReturnsBadRequest()
        {
            // Arrange
            var loginDto = new LoginDto
            {
                Email = null,
                Password = "Password123!"
            };

            // Act
            var result = await _controller.Login(loginDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        [Fact]
        public async Task Login_WithNullPassword_ReturnsBadRequest()
        {
            // Arrange
            var loginDto = new LoginDto
            {
                Email = "test@example.com",
                Password = null
            };

            // Act
            var result = await _controller.Login(loginDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        [Fact]
        public async Task Login_WhenUserNotFound_ReturnsBadRequest()
        {
            // Arrange
            var loginDto = new LoginDto
            {
                Email = "nonexistent@example.com",
                Password = "Password123!"
            };

            _mockAccountService
                .Setup(s => s.SignInAsync(loginDto.Email, loginDto.Password))
                .ReturnsAsync(IdentitySignInResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(loginDto.Email))
                .ReturnsAsync((Account?)null);

            // Act
            var result = await _controller.Login(loginDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        #endregion

        #region Create Account Tests

        [Fact]
        public async Task Create_WithValidAdminDto_ReturnsOkWithUserData()
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "Admin User",
                Email = "admin@example.com",
                Password = "Password123!",
                SelectedRole = "ADMIN"
            };

            var createdUser = new Account
            {
                Id = "admin-123",
                Email = "admin@example.com",
                UserName = "admin@example.com",
                FullName = "Admin User",
                EmailConfirmed = false,
                Role = "ADMIN"
            };

            _mockAccountService
                .Setup(s => s.RegisterAdminAsync(createDto))
                .ReturnsAsync(IdentityResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(createDto.Email))
                .ReturnsAsync(createdUser);

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var okResult = Assert.IsType<OkObjectResult>(result);
            Assert.Equal(200, okResult.StatusCode);

            var responseValue = okResult.Value;
            Assert.NotNull(responseValue);

            var messageProperty = responseValue.GetType().GetProperty("Message");
            Assert.NotNull(messageProperty);
            Assert.Equal("Creation Success", messageProperty.GetValue(responseValue));
        }

        [Fact]
        public async Task Create_WithValidStudentDto_ReturnsOkWithUserData()
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "Student User",
                Email = "student@example.com",
                Password = "Password123!",
                SelectedRole = "STUDENT"
            };

            var createdUser = new Account
            {
                Id = "student-123",
                Email = "student@example.com",
                UserName = "student@example.com",
                FullName = "Student User",
                EmailConfirmed = false,
                Role = "STUDENT"
            };

            _mockAccountService
                .Setup(s => s.RegisterStudentAsync(createDto))
                .ReturnsAsync(IdentityResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(createDto.Email))
                .ReturnsAsync(createdUser);

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var okResult = Assert.IsType<OkObjectResult>(result);
            Assert.Equal(200, okResult.StatusCode);
        }

        [Fact]
        public async Task Create_WithValidSLPDto_ReturnsOkWithUserData()
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "SLP User",
                Email = "slp@example.com",
                Password = "Password123!",
                SelectedRole = "SLP"
            };

            var createdUser = new Account
            {
                Id = "slp-123",
                Email = "slp@example.com",
                UserName = "slp@example.com",
                FullName = "SLP User",
                EmailConfirmed = false,
                Role = "SLP"
            };

            _mockAccountService
                .Setup(s => s.RegisterSLPAsync(createDto))
                .ReturnsAsync(IdentityResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(createDto.Email))
                .ReturnsAsync(createdUser);

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var okResult = Assert.IsType<OkObjectResult>(result);
            Assert.Equal(200, okResult.StatusCode);
        }

        [Fact]
        public async Task Create_WithInvalidRole_ReturnsBadRequest()
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "Invalid User",
                Email = "invalid@example.com",
                Password = "Password123!",
                SelectedRole = "INVALID_ROLE"
            };

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        [Fact]
        public async Task Create_WhenRegistrationFails_ReturnsBadRequest()
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "Test User",
                Email = "test@example.com",
                Password = "Password123!",
                SelectedRole = "STUDENT"
            };

            var identityError = new IdentityError
            {
                Code = "DuplicateEmail",
                Description = "Email already exists."
            };

            _mockAccountService
                .Setup(s => s.RegisterStudentAsync(createDto))
                .ReturnsAsync(IdentityResult.Failed(identityError));

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        [Fact]
        public async Task Create_WhenCreatedUserNotFound_ReturnsBadRequest()
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "Test User",
                Email = "test@example.com",
                Password = "Password123!",
                SelectedRole = "STUDENT"
            };

            _mockAccountService
                .Setup(s => s.RegisterStudentAsync(createDto))
                .ReturnsAsync(IdentityResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(createDto.Email))
                .ReturnsAsync((Account?)null);

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var badRequestResult = Assert.IsType<BadRequestObjectResult>(result);
            Assert.Equal(400, badRequestResult.StatusCode);
        }

        [Theory]
        [InlineData("admin")]
        [InlineData("Admin")]
        [InlineData("ADMIN")]
        public async Task Create_WithRoleInDifferentCase_RouteToCorrectRegistration(string role)
        {
            // Arrange
            var createDto = new CreateAccountDto
            {
                FullName = "Test Admin",
                Email = "admin@example.com",
                Password = "Password123!",
                SelectedRole = role
            };

            var createdUser = new Account
            {
                Id = "admin-123",
                Email = "admin@example.com",
                UserName = "admin@example.com",
                FullName = "Test Admin",
                EmailConfirmed = false,
                Role = "ADMIN"
            };

            _mockAccountService
                .Setup(s => s.RegisterAdminAsync(createDto))
                .ReturnsAsync(IdentityResult.Success);

            _mockAccountService
                .Setup(s => s.GetByEmailAsync(createDto.Email))
                .ReturnsAsync(createdUser);

            // Act
            var result = await _controller.Create(createDto);

            // Assert
            var okResult = Assert.IsType<OkObjectResult>(result);
            Assert.Equal(200, okResult.StatusCode);

            _mockAccountService.Verify(
                s => s.RegisterAdminAsync(It.IsAny<CreateAccountDto>()),
                Times.Once);
        }

        #endregion
    }
}
