# Bloom Test Suite

Unit and integration tests for the Bloom backend, using xUnit + Moq. Runs standalone against
the .NET SDK -- no Docker or MariaDB required, including for the integration tests (see below).

## Project Structure

```
tests/
├── Services/
│   ├── AccountServiceTests.cs         # AccountService: auth, registration, role checks, delete
│   ├── RsrSpeechServiceTests.cs       # RsrSpeechService: manifest loading, queue/pending/ack
│   ├── RepeatRequestDetectorTests.cs  # RepeatRequestDetector: phrase-matching for "please repeat"
│   └── RobotSessionServiceTests.cs    # RobotSessionService: session/lesson lifecycle logic
├── Integration/
│   ├── BloomWebApplicationFactory.cs  # Real ASP.NET Core pipeline + SQLite in-memory DB
│   ├── AuthTestHelper.cs              # Mints a JWT for an authenticated HttpClient
│   ├── TestJson.cs                    # Shared JsonSerializerOptions (camelCase, case-insensitive)
│   ├── RobotSessionControllerTests.cs
│   ├── LessonRuntimeControllerTests.cs
│   └── LessonSessionControllerTests.cs
├── TestHelpers/
│   └── TestDataSeeder.cs              # Builds the minimum valid EF graph for a test
└── bloom.Tests.csproj
```

## Running Tests

### Prerequisites

- .NET 9.0 SDK ([download](https://dotnet.microsoft.com/download/dotnet/9.0))

### Run All Tests
```bash
dotnet test tests/bloom.Tests.csproj
```

### Run a Specific Test Class
```bash
dotnet test tests/bloom.Tests.csproj --filter ClassName~LessonRuntimeControllerTests
```

### Run with Verbose Output
```bash
dotnet test tests/bloom.Tests.csproj -v n
```

### Generate a Coverage Report (requires coverlet)
```bash
dotnet test tests/bloom.Tests.csproj /p:CollectCoverage=true /p:CoverageFormat=opencover
```

### CI
`.github/workflows/ci.yml` runs this project's `dotnet test` (and the frontend's `npm test`)
on every push/PR to `main`/`development`. That workflow is the source of truth for what
actually gates a merge -- `test.sh` at the repo root remains available for local/Jenkins runs
that spin up the full Docker Compose stack (a real MariaDB, not SQLite).

## Integration tests: how `BloomWebApplicationFactory` works

`BloomWebApplicationFactory` boots the real ASP.NET Core pipeline (`Program.cs`, all
middleware/controllers/DI as registered in production) but swaps `BloomDbContext` to a SQLite
connection **held open for the lifetime of the factory instance**. A bare
`"DataSource=:memory:"` connection string creates a fresh, empty database every time EF opens a
connection; holding one `SqliteConnection` open (and passing that object to `UseSqlite`) is what
keeps every `DbContext` scope created by an HTTP request talking to the same in-memory database.

The checked-in EF Core migrations are MySQL/Pomelo-flavored (the real deployment's provider), so
`Program.cs` branches on the DbContext's provider name at startup: SQLite gets
`Database.EnsureCreated()` (schema built straight from the current model, no migration history
involved), everything else still gets `Database.Migrate()` exactly as before. This is a
test-only code path -- real (MySQL) deployments are unaffected.

**One factory instance is created per test class** via `IClassFixture<BloomWebApplicationFactory>`,
so each test class gets its own isolated database *and* its own instances of the services
registered as DI singletons in `Program.cs` (`IStepControlService`, `IRobotStateRepository`,
`IRepeatRequestDetector`) -- none of that state crosses test-class boundaries. Tests **within**
one class share both the database and those singletons, so **always seed fresh data per test
method** (via `TestDataSeeder`, which mints new Guids/emails on every call) rather than relying
on data set up by a different test in the same class.

Authenticate as a seeded account with `AuthTestHelper.CreateAuthorizedClient(factory, account)`,
which mints a minimal JWT rather than driving a full HTTP login per test.

## A note on `LessonSessionController` vs `LessonRuntimeController`

These two controllers currently cover nearly the same route surface (start/stop/skip/replay/
set-step/progress/step-control/history) -- `LessonRuntimeController` is the newer, actively
maintained one and is where most step-control/lesson-lifecycle logic has landed; this looks like
an in-progress consolidation. Test coverage is concentrated on `LessonRuntimeController`
(`LessonRuntimeControllerTests.cs`); `LessonSessionControllerTests.cs` only covers the one real
behavioral difference (`[Authorize(Policy = "JwtOrCookie")]`) plus the couple of behaviors both
controllers independently implement, rather than duplicating full coverage.

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
    var result = await _service.Method(testData);

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

- Prefer real objects over mocks where they're cheap (a real `BloomDbContext` against SQLite
  in-memory, real trivial repositories/services like `SessionCodeService`) -- reserve Moq for
  dependencies that are expensive, external, or where you specifically want a `Verify()`
  assertion on exact call behavior.
- Use `Times.Once`, `Times.Never` for verifying mock calls.
- Tests are isolated and can run in any order within a class; different integration test classes
  run safely in parallel (see above).

## Future Enhancements

- [x] Integration tests with in-memory database
- [x] Tests for robot session management
- [x] Tests for lesson coordination
- [ ] Frontend component/unit tests (React Testing Library, MSW for API mocking)
- [ ] Browser E2E tests for the main frontend (a Playwright suite already exists for the
      RSR playback flow in `robot/kiosk-face/apps/tutorial-fullscreen-face/e2e/` as a model)
- [ ] Broader unit-test coverage across the remaining controllers/services untouched by this pass
- [ ] Performance benchmarking tests
