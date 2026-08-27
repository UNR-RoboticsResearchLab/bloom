using System.Net;
using System.Net.Http;
using System.Net.Http.Json;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// LessonSessionController duplicates LessonRuntimeController's route surface almost 1:1
    /// (looks like an in-progress consolidation onto LessonRuntimeController -- worth flagging
    /// to the team, not resolved here). This file covers only the one real behavioral
    /// difference ([Authorize(Policy = "JwtOrCookie")], unlike LessonRuntimeController) plus the
    /// two behaviors changed in this pass that this controller independently implements
    /// (set-step bounds check, Completed-but-still-controllable lifecycle). See
    /// LessonRuntimeControllerTests.cs for full coverage of the shared RobotSessionService logic.
    /// </summary>
    public class LessonSessionControllerTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public LessonSessionControllerTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private sealed record Fixture(HttpClient Client, Account Slp, Lesson Lesson, RobotSession Session);

        private async Task<Fixture> SeedAsync(int totalSteps = 3)
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var (slp, _, lesson, session) =
                await TestDataSeeder.SeedLessonRuntimeGraphAsync(db, users, totalSteps);
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);
            return new Fixture(client, slp, lesson, session);
        }

        private static async Task StartLessonAsync(Fixture f)
        {
            var resp = await f.Client.PostAsJsonAsync($"api/LessonSession/{f.Session.Id}/lesson",
                new StartLessonDto { LessonId = f.Lesson.Id, StudentId = f.Slp.Id });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        }

        [Fact]
        public async Task Unauthenticated_PendingLessonPoll_Returns401()
        {
            var f = await SeedAsync();
            var anonymousClient = _factory.CreateClient();

            var resp = await anonymousClient.GetAsync($"api/LessonSession/{f.Session.Id}/pending-lesson");

            Assert.Equal(HttpStatusCode.Unauthorized, resp.StatusCode);
        }

        [Fact]
        public async Task SetStep_TargetStepAboveTotal_ReturnsBadRequest()
        {
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/LessonSession/{f.Session.Id}/lessons/set-step",
                new SetStepDto { TargetStep = 4 });

            Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        }

        [Fact]
        public async Task SetStep_TargetStepEqualsTotalSteps_Succeeds()
        {
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/LessonSession/{f.Session.Id}/lessons/set-step",
                new SetStepDto { TargetStep = 3 });

            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        }

        [Fact]
        public async Task UpdateLessonProgress_StatusCompleted_KeepsActiveLessonIdSet()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PutAsJsonAsync($"api/LessonSession/{f.Session.Id}/lessons/progress",
                new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var getResp = await f.Client.GetAsync($"api/RobotSession/{f.Session.Id}");
            var body = await getResp.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Equal(f.Lesson.Id, body!.ActiveLessonId);
        }

        [Fact]
        public async Task StartLesson_ThenStop_ClearsActiveLesson()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var stopResp = await f.Client.PostAsync($"api/LessonSession/{f.Session.Id}/lessons/stop", null);
            Assert.Equal(HttpStatusCode.OK, stopResp.StatusCode);

            var getResp = await f.Client.GetAsync($"api/RobotSession/{f.Session.Id}");
            var body = await getResp.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Null(body!.ActiveLessonId);
        }
    }
}
