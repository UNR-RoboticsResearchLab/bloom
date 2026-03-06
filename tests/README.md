# Bloom Test Suite

This project contains unit and integration tests for the Bloom backend using xUnit and Moq.

## Project Structure

```
bloom.Tests/
├── Controllers/
│   └── AccountsControllerTests.cs     # Tests for account creation, login, and profile management
├── Services/
│   └── AccountServiceTests.cs         # Tests for authentication and user management
└── bloom.Tests.csproj                 # Test project configuration
```

## Running Tests

### Prerequisites

- .NET 9.0 SDK properly installed ([download](https://dotnet.microsoft.com/download/dotnet/9.0))
- Visual Studio, VS Code with C# extension, or JetBrains Rider
- All NuGet packages restored

### Troubleshooting .NET SDK Issues

If you see "No .NET SDKs were found":

1. Verify installation: `dotnet --version`
2. Check SDK location: `dotnet --list-sdks`
3. Reinstall .NET 9.0 if needed from the download link above

### Run All Tests
```bash
dotnet test bloom.Tests/bloom.Tests.csproj
```

### Run Specific Test Class
```bash
dotnet test bloom.Tests/bloom.Tests.csproj --filter ClassName=AccountsControllerTests
```

### Run with Verbose Output
```bash
dotnet test bloom.Tests/bloom.Tests.csproj -v n
```

### Generate Coverage Report (requires coverlet)
```bash
dotnet test bloom.Tests/bloom.Tests.csproj /p:CollectCoverage=true /p:CoverageFormat=opencover
```

## Test Coverage

### AccountsControllerTests
Tests the AccountsController endpoints for:
- **Login endpoint** (`POST /api/accounts/login`)
  - ✅ Valid credentials return OK with user data
  - ✅ Invalid credentials return BadRequest
  - ✅ Null email returns BadRequest
  - ✅ Null password returns BadRequest
  - ✅ User not found returns BadRequest

- **Create account endpoint** (`POST /api/accounts/create`)
  - ✅ Valid ADMIN creation returns OK
  - ✅ Valid STUDENT creation returns OK
  - ✅ Valid SLP creation returns OK
  - ✅ Invalid role returns BadRequest
  - ✅ Registration failure returns BadRequest
  - ✅ Created user not found returns BadRequest
  - ✅ Case-insensitive role matching

### AccountServiceAuthTests
Tests the AccountService authentication methods:
- **SignInAsync**
  - ✅ Valid credentials succeed
  - ✅ Invalid password fails

- **CreateAsync**
  - ✅ Valid user creation succeeds
  - ✅ Weak password fails with appropriate error

- **FindByEmailAsync / FindByIdAsync**
  - ✅ Existing user found
  - ✅ Non-existent user returns null

- **Role checks**
  - ✅ IsInRoleAsync returns true when user is in role
  - ✅ IsInRoleAsync returns false when user is not in role

- **DeleteAsync**
  - ✅ Valid user deletion succeeds
  - ✅ Deletion failure returns failed result

## Dependencies

- **xunit** - Test framework
- **Moq** - Mocking library
- **Microsoft.NET.Test.Sdk** - Test runner
- **Microsoft.AspNetCore.Mvc.Testing** - Integration testing support
- **Microsoft.AspNetCore.Identity** - Identity framework

## Writing New Tests

### Example Test Structure
```csharp
[Fact]
public async Task MethodName_WithCondition_ExpectedResult()
{
    // Arrange
    var testData = new TestData();
    _mockService.Setup(s => s.Method(It.IsAny<string>()))
        .ReturnsAsync(expectedResult);

    // Act
    var result = await _controller.Method(testData);

    // Assert
    Assert.NotNull(result);
    Assert.Equal(expectedValue, result.Value);
}
```

### Naming Convention
- Test method names follow: `MethodName_WithCondition_ExpectedResult`
- Test classes end with `Tests`
- Use `[Fact]` for regular tests
- Use `[Theory]` with `[InlineData]` for parameterized tests

## Notes

- Tests use Moq for mocking dependencies
- Controller tests mock the IAccountService interface
- Service tests mock UserManager and SignInManager
- Tests are isolated and can run in any order
- Use `Times.Once`, `Times.Never` for verifying mock calls

## Future Enhancements

- [ ] Integration tests with in-memory database
- [ ] Tests for robot session management
- [ ] Tests for lesson coordination
- [ ] Performance benchmarking tests
- [ ] E2E tests with WebApplicationFactory
