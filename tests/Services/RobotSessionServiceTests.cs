using System;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Repositories;
using bloom.Services;
using bloom.Tests.TestHelpers;
using Microsoft.Data.Sqlite;
using Microsoft.EntityFrameworkCore;
using Moq;
using Xunit;

namespace bloom.Tests.Services
{
    /// <summary>
    /// Unit tests for RobotSessionService's session/lesson lifecycle logic -- a second line of
    /// defense under RobotSessionControllerTests/LessonRuntimeControllerTests, targeting the
    /// same new/changed behavior with Moq Verify() assertions that are strictly stronger than
    /// "poll and observe an eventual side effect."
    ///
    /// Uses a real SQLite in-memory BloomDbContext (held-open connection, same pattern as
    /// BloomWebApplicationFactory) plus the real RobotSessionRepository/SessionCodeService
    /// (trivial DB wrappers, cheaper to use for real than to mock), and Moq for
    /// IRobotStateRepository/IStepControlService.
    /// </summary>
    public class RobotSessionServiceTests : IDisposable
    {
        private readonly SqliteConnection _connection;
        private readonly BloomDbContext _db;
        private readonly Mock<IRobotStateRepository> _stateRepo = new();
        private readonly Mock<IStepControlService> _stepControl = new();
        private readonly RobotSessionService _service;

        public RobotSessionServiceTests()
        {
            _connection = new SqliteConnection("DataSource=:memory:");
            _connection.Open();

            var options = new DbContextOptionsBuilder<BloomDbContext>()
                .UseSqlite(_connection)
                .Options;
            _db = new BloomDbContext(options);
            _db.Database.EnsureCreated();

            var sessionRepo = new RobotSessionRepository(_db);
            var codeService = new SessionCodeService(_db);

            _service = new RobotSessionService(_db, sessionRepo, _stateRepo.Object, codeService, _stepControl.Object);
        }

        public void Dispose()
        {
            _db.Dispose();
            _connection.Dispose();
        }

        private (Account Slp, Robot Robot, Lesson Lesson) SeedGraph(int totalSteps = 3)
        {
            var slp = TestDataSeeder.CreateRawAccount(_db, role: "SLP");
            var robot = TestDataSeeder.CreateRobot(_db);
            var lesson = TestDataSeeder.CreateLessonWithSteps(_db, slp.Id, totalSteps);
            return (slp, robot, lesson);
        }

        private async Task<RobotSession> SeedSessionAsync(string? userId = null)
        {
            var session = new RobotSession
            {
                UserId = userId,
                SessionCode = Random.Shared.Next(100000, 999999).ToString(),
                CreatedAt = DateTime.UtcNow,
                LastUpdatedAt = DateTime.UtcNow
            };
            _db.RobotSessions.Add(session);
            await _db.SaveChangesAsync();
            return session;
        }

        // ---- EndSessionAsync ------------------------------------------------------------------

        [Fact]
        public async Task EndSessionAsync_ClearsStepControlService()
        {
            var session = await SeedSessionAsync();

            await _service.EndSessionAsync(session.Id);

            _stepControl.Verify(s => s.ClearControl(session.Id), Times.Once);
        }

        [Fact]
        public async Task EndSessionAsync_ClosesOpenLessonRunAsAbandoned()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            var run = new LessonRun { RobotSessionId = session.Id, LessonId = lesson.Id, SlpId = slp.Id, Status = "active" };
            _db.LessonRuns.Add(run);
            session.ActiveLessonId = lesson.Id;
            session.ActiveLessonRunId = run.Id;
            await _db.SaveChangesAsync();

            await _service.EndSessionAsync(session.Id);

            var reloaded = await _db.LessonRuns.FindAsync(run.Id);
            Assert.Equal("abandoned", reloaded!.Status);
            Assert.NotNull(reloaded.EndedAt);
        }

        [Fact]
        public async Task EndSessionAsync_AlreadyClosedRun_DoesNotOverwriteEndedAtOrStatus()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            var closedAt = DateTime.UtcNow.AddMinutes(-10);
            var run = new LessonRun
            {
                RobotSessionId = session.Id,
                LessonId = lesson.Id,
                SlpId = slp.Id,
                Status = "completed",
                EndedAt = closedAt
            };
            _db.LessonRuns.Add(run);
            session.ActiveLessonId = lesson.Id;
            session.ActiveLessonRunId = run.Id;
            await _db.SaveChangesAsync();

            await _service.EndSessionAsync(session.Id);

            var reloaded = await _db.LessonRuns.FindAsync(run.Id);
            Assert.Equal("completed", reloaded!.Status);
            Assert.Equal(closedAt, reloaded.EndedAt);
        }

        // ---- StartLessonAsync -----------------------------------------------------------------

        [Fact]
        public async Task StartLessonAsync_ClearsStepControlUnconditionally()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);

            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });

            _stepControl.Verify(s => s.ClearControl(session.Id), Times.Once);
        }

        [Fact]
        public async Task StartLessonAsync_AbandonsStillOpenPriorRun_AndCreatesNewActiveRun()
        {
            var (slp, _, lessonA) = SeedGraph();
            var lessonB = TestDataSeeder.CreateLessonWithSteps(_db, slp.Id, 2);
            var session = await SeedSessionAsync(slp.Id);

            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lessonA.Id, StudentId = slp.Id });
            var firstRunId = (await _db.RobotSessions.FindAsync(session.Id))!.ActiveLessonRunId;

            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lessonB.Id, StudentId = slp.Id });
            var reloadedSession = await _db.RobotSessions.FindAsync(session.Id);

            var firstRun = await _db.LessonRuns.FindAsync(firstRunId);
            var secondRun = await _db.LessonRuns.FindAsync(reloadedSession!.ActiveLessonRunId);

            Assert.Equal("abandoned", firstRun!.Status);
            Assert.NotNull(firstRun.EndedAt);
            Assert.Equal("active", secondRun!.Status);
            Assert.Equal(lessonB.Id, reloadedSession.ActiveLessonId);
        }

        [Fact]
        public async Task StartLessonAsync_EmptyLessonId_ThrowsArgumentException()
        {
            var (slp, _, _) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);

            await Assert.ThrowsAsync<ArgumentException>(() =>
                _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = Guid.Empty, StudentId = slp.Id }));
        }

        // ---- GetActiveLessonTotalStepsAsync -----------------------------------------------------

        [Fact]
        public async Task GetActiveLessonTotalStepsAsync_NoActiveLesson_ReturnsNull()
        {
            var session = await SeedSessionAsync();

            var result = await _service.GetActiveLessonTotalStepsAsync(session.Id);

            Assert.Null(result);
        }

        [Fact]
        public async Task GetActiveLessonTotalStepsAsync_ReturnsSeededLessonTotalSteps()
        {
            var (slp, _, lesson) = SeedGraph(totalSteps: 5);
            var session = await SeedSessionAsync(slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });

            var result = await _service.GetActiveLessonTotalStepsAsync(session.Id);

            Assert.Equal(5, result);
        }

        // ---- UpdateLessonProgressAsync ---------------------------------------------------------

        [Fact]
        public async Task UpdateLessonProgressAsync_StatusCompleted_DoesNotClearActiveLessonId()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });

            await _service.UpdateLessonProgressAsync(session.Id,
                new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" });

            var reloaded = await _db.RobotSessions.FindAsync(session.Id);
            Assert.Equal(lesson.Id, reloaded!.ActiveLessonId);
            Assert.NotNull(reloaded.ActiveLessonRunId);
        }

        [Fact]
        public async Task UpdateLessonProgressAsync_StatusCompleted_ClosesRunAndMarksAssignmentCompleted()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            TestDataSeeder.CreateAssignment(_db, studentId: slp.Id, lessonId: lesson.Id, assignedById: slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });

            await _service.UpdateLessonProgressAsync(session.Id,
                new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" });

            var run = await _db.LessonRuns.FirstAsync(r => r.RobotSessionId == session.Id);
            var assignment = await _db.Assignments.FirstAsync(a => a.StudentId == slp.Id && a.LessonId == lesson.Id);
            Assert.Equal("completed", run.Status);
            Assert.NotNull(run.EndedAt);
            Assert.True(assignment.IsCompleted);
        }

        [Fact]
        public async Task UpdateLessonProgressAsync_StatusCompleted_CalledTwice_IsIdempotent()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });
            var dto = new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" };

            await _service.UpdateLessonProgressAsync(session.Id, dto);
            var endedAtAfterFirst = (await _db.LessonRuns.FirstAsync(r => r.RobotSessionId == session.Id)).EndedAt;

            await _service.UpdateLessonProgressAsync(session.Id, dto);
            var endedAtAfterSecond = (await _db.LessonRuns.FirstAsync(r => r.RobotSessionId == session.Id)).EndedAt;

            Assert.Equal(endedAtAfterFirst, endedAtAfterSecond);
        }

        [Fact]
        public async Task UpdateLessonProgressAsync_NoActiveLesson_ThrowsInvalidOperationException()
        {
            var session = await SeedSessionAsync();

            await Assert.ThrowsAsync<InvalidOperationException>(() =>
                _service.UpdateLessonProgressAsync(session.Id,
                    new UpdateLessonProgressDto { CurrentStepId = 1, CompletedSteps = 1, Status = "InProgress" }));
        }

        [Fact]
        public async Task UpdateLessonProgressAsync_StatusInProgress_ComputesPercentageFromTotalSteps()
        {
            var (slp, _, lesson) = SeedGraph(totalSteps: 4);
            var session = await SeedSessionAsync(slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });

            await _service.UpdateLessonProgressAsync(session.Id,
                new UpdateLessonProgressDto { CurrentStepId = 1, CompletedSteps = 1, Status = "InProgress" });

            var progress = await _db.LessonProgresses.FirstAsync(p => p.LessonId == lesson.Id && p.StudentId == slp.Id);
            Assert.Equal(25, progress.ProgressPercentage);
        }

        // ---- StopLessonAsync ------------------------------------------------------------------

        [Fact]
        public async Task StopLessonAsync_ClearsActiveLessonAndAbandonsRun()
        {
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });

            await _service.StopLessonAsync(session.Id);

            var reloaded = await _db.RobotSessions.FindAsync(session.Id);
            Assert.Null(reloaded!.ActiveLessonId);
            Assert.Null(reloaded.ActiveLessonRunId);

            var run = await _db.LessonRuns.FirstAsync(r => r.RobotSessionId == session.Id);
            Assert.Equal("abandoned", run.Status);
        }

        [Fact]
        public async Task StopLessonAsync_NeverTouchesStepControlService()
        {
            // StopLessonAsync deliberately leaves the queued "stop" command in place for the
            // robot to poll and self-acknowledge -- it's the CONTROLLER (not the service) that
            // queues "stop" via IStepControlService.SetPendingControl. This is an easy-to-miss
            // asymmetry with EndSessionAsync/StartLessonAsync, which proactively clear
            // step-control themselves; pinning it here so a future "helpful" refactor that adds
            // a ClearControl call to StopLessonAsync gets caught.
            var (slp, _, lesson) = SeedGraph();
            var session = await SeedSessionAsync(slp.Id);
            await _service.StartLessonAsync(session.Id, new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });
            _stepControl.Invocations.Clear();

            await _service.StopLessonAsync(session.Id);

            _stepControl.Verify(s => s.ClearControl(It.IsAny<Guid>()), Times.Never);
        }
    }
}
