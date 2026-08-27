using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Http;
using System.Net.Http.Json;
using System.Text.Json;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Controller-level integration tests for RobotSessionController, run against a real
    /// ASP.NET Core pipeline (BloomWebApplicationFactory) with a SQLite in-memory database.
    /// One BloomWebApplicationFactory instance (and thus one database) is shared across every
    /// test in this class via IClassFixture -- each test seeds its own fresh data.
    /// </summary>
    public class RobotSessionControllerTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;
        private readonly HttpClient _client;

        public RobotSessionControllerTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
            _client = factory.CreateClient();
        }

        [Fact]
        public async Task CreateSession_Anonymous_ReturnsCreatedWithNullUserId()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robot = TestDataSeeder.CreateRobot(db);

            var response = await _client.PostAsJsonAsync("api/RobotSession",
                new StartSessionDto { RobotId = robot.Id, Anonymous = true });

            Assert.Equal(HttpStatusCode.Created, response.StatusCode);
            var body = await response.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.NotNull(body);
            Assert.Null(body!.UserId);
            Assert.Equal(6, body.SessionCode?.Length);
            Assert.Contains(robot.Id, body.RobotIds);
        }

        [Fact]
        public async Task CreateSession_UnknownRobot_ReturnsNotFound()
        {
            var response = await _client.PostAsJsonAsync("api/RobotSession",
                new StartSessionDto { RobotId = Guid.NewGuid(), Anonymous = true });

            Assert.Equal(HttpStatusCode.NotFound, response.StatusCode);
        }

        [Fact]
        public async Task GetSession_ExistingSession_ReturnsSessionDetails()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot);

            var response = await _client.GetAsync($"api/RobotSession/{session.Id}");

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            var body = await response.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Equal(session.Id, body!.Id);
        }

        [Fact]
        public async Task GetSession_UnknownId_ReturnsNotFound()
        {
            var response = await _client.GetAsync($"api/RobotSession/{Guid.NewGuid()}");
            Assert.Equal(HttpStatusCode.NotFound, response.StatusCode);
        }

        [Fact]
        public async Task JoinSession_ByCode_AuthenticatedUser_SetsSessionUserId()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot); // anonymous (UserId null)
            var joiner = await TestDataSeeder.CreateAccountAsync(users, role: "Student");
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, joiner);

            var response = await client.GetAsync($"api/RobotSession/join/{session.SessionCode}");

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            var body = await response.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Equal(joiner.Id, body!.UserId);

            // Confirm it actually persisted, not just echoed back in this one response.
            var followUp = await _client.GetAsync($"api/RobotSession/{session.Id}");
            var followUpBody = await followUp.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Equal(joiner.Id, followUpBody!.UserId);
        }

        [Fact]
        public async Task EndSession_AsNonOwner_ReturnsForbidden()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var owner = await TestDataSeeder.CreateAccountAsync(users, role: "SLP");
            var intruder = await TestDataSeeder.CreateAccountAsync(users, role: "SLP");
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot, userId: owner.Id);

            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, intruder);
            var response = await client.PostAsync($"api/RobotSession/{session.Id}/end", null);

            Assert.Equal(HttpStatusCode.Forbidden, response.StatusCode);
        }

        [Fact]
        public async Task EndSession_AsOwner_ClearsStepControlAndAbandonsRun()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var (slp, _, lesson, session) = await TestDataSeeder.SeedLessonRuntimeGraphAsync(db, users);
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);

            var startResp = await client.PostAsJsonAsync($"api/lesson-runtime/{session.Id}/start",
                new StartLessonDto { LessonId = lesson.Id, StudentId = slp.Id });
            Assert.Equal(HttpStatusCode.OK, startResp.StatusCode);

            var skipResp = await client.PostAsync($"api/lesson-runtime/{session.Id}/skip", null);
            Assert.Equal(HttpStatusCode.OK, skipResp.StatusCode);

            var endResp = await client.PostAsync($"api/RobotSession/{session.Id}/end", null);
            Assert.Equal(HttpStatusCode.OK, endResp.StatusCode);

            // Step-control was cleared: nothing left for the robot to poll for.
            var pollResp = await client.GetAsync($"api/lesson-runtime/{session.Id}/step-control");
            Assert.Equal(HttpStatusCode.NoContent, pollResp.StatusCode);

            // The still-open LessonRun was closed out as abandoned, not left dangling.
            using var verifyScope = _factory.Services.CreateScope();
            var verifyDb = verifyScope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var run = await verifyDb.LessonRuns.FirstAsync(r => r.RobotSessionId == session.Id);
            Assert.Equal("abandoned", run.Status);
            Assert.NotNull(run.EndedAt);
        }

        [Fact]
        public async Task AddRobot_UnknownSession_ReturnsNotFound()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robot = TestDataSeeder.CreateRobot(db);

            var response = await _client.PostAsJsonAsync($"api/RobotSession/{Guid.NewGuid()}/robots",
                new StartSessionDto { RobotId = robot.Id });

            Assert.Equal(HttpStatusCode.NotFound, response.StatusCode);
        }

        [Fact]
        public async Task RemoveRobot_DecrementsRobotCount()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot); // seeded with Robots = 1

            var response = await _client.DeleteAsync($"api/RobotSession/{session.Id}/robots/{robot.Id}");
            Assert.Equal(HttpStatusCode.OK, response.StatusCode);

            using var verifyScope = _factory.Services.CreateScope();
            var verifyDb = verifyScope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var updated = await verifyDb.RobotSessions.FindAsync(session.Id);
            Assert.Equal(0, updated!.Robots);
        }

        [Fact]
        public async Task UpdateRobotState_UnknownSession_ReturnsNotFound()
        {
            var response = await _client.PutAsJsonAsync(
                $"api/RobotSession/{Guid.NewGuid()}/robots/{Guid.NewGuid()}/state",
                new RobotStateDto { Status = "idle" });

            Assert.Equal(HttpStatusCode.NotFound, response.StatusCode);
        }

        [Fact]
        public async Task UnpairUser_ClearsSessionUserId()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var owner = await TestDataSeeder.CreateAccountAsync(users, role: "SLP");
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot, userId: owner.Id);

            var response = await _client.DeleteAsync($"api/RobotSession/{session.Id}/user");
            Assert.Equal(HttpStatusCode.OK, response.StatusCode);

            var getResp = await _client.GetAsync($"api/RobotSession/{session.Id}");
            var body = await getResp.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Null(body!.UserId);
        }

        [Fact]
        public async Task GetSessionHistory_ReturnsWellFormedList()
        {
            // Not asserting an exact row count: InMemoryRobotStateRepository.Add() fire-and-forgets
            // its own RobotStateHistory write on every robot-state update, independent of (and in
            // addition to) RobotSessionService's own "archive every 10th change" path -- exact
            // counts are racy against that unawaited background write.
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot);

            var response = await _client.GetAsync($"api/RobotSession/{session.Id}/history");

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            var body = await response.Content.ReadFromJsonAsync<JsonElement>(TestJson.Options);
            Assert.True(body.TryGetProperty("history", out var historyProp));
            Assert.Equal(JsonValueKind.Array, historyProp.ValueKind);
        }

        [Fact]
        public async Task GetTrackerEvents_IncludesSessionCreatedEvent()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robot = TestDataSeeder.CreateRobot(db);
            var session = await TestDataSeeder.CreateSessionAsync(db, robot);

            var response = await _client.GetAsync($"api/RobotSession/{session.Id}/tracker-events");

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            var events = await response.Content.ReadFromJsonAsync<List<TrackerEventDto>>(TestJson.Options);
            Assert.NotNull(events);
            Assert.Contains(events!, e => e.EventType == "session_created");
        }
    }
}
