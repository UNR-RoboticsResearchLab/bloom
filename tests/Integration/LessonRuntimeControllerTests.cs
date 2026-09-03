using System;
using System.Net;
using System.Net.Http;
using System.Net.Http.Json;
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
    /// Controller-level integration tests for LessonRuntimeController -- the canonical,
    /// actively-developed controller for real-time lesson activity. This is the primary
    /// regression coverage for the branch's in-flight changes: the set-step bounds check, the
    /// "Completed but still controllable" lesson-lifecycle change, repeat-request-triggers-replay,
    /// and stale step-control cleanup on lesson restart.
    /// </summary>
    public class LessonRuntimeControllerTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public LessonRuntimeControllerTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private sealed record Fixture(HttpClient Client, Account Slp, Robot Robot, Lesson Lesson, RobotSession Session);

        /// <summary>
        /// Seeds an SLP-owned session with a `totalSteps`-step lesson and returns an HttpClient
        /// authorized as that SLP. The session's UserId is the SLP's own account -- this is the
        /// "SLP creates and owns the session directly" flow (as opposed to a student joining by
        /// pairing code), which keeps StartLessonAsync's ownership check satisfied for free.
        /// </summary>
        private async Task<Fixture> SeedAsync(int totalSteps = 3)
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var (slp, robot, lesson, session) =
                await TestDataSeeder.SeedLessonRuntimeGraphAsync(db, users, totalSteps);
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);
            return new Fixture(client, slp, robot, lesson, session);
        }

        private static async Task StartLessonAsync(Fixture f, Guid? lessonId = null)
        {
            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/start",
                new StartLessonDto { LessonId = lessonId ?? f.Lesson.Id, StudentId = f.Slp.Id });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        }

        // ---- set-step bounds check (new behavior) -------------------------------------------

        [Fact]
        public async Task SetStep_WithinBounds_QueuesSetStepCommand()
        {
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/set-step",
                new SetStepDto { TargetStep = 2 });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            Assert.Equal(HttpStatusCode.OK, poll.StatusCode);
            var control = await poll.Content.ReadFromJsonAsync<PendingStepControlDto>(TestJson.Options);
            Assert.Equal("set_step", control!.Command);
            Assert.Equal(2, control.TargetStep);
        }

        [Fact]
        public async Task SetStep_TargetStepZero_ReturnsBadRequest()
        {
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/set-step",
                new SetStepDto { TargetStep = 0 });

            Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        }

        [Fact]
        public async Task SetStep_TargetStepAboveTotal_ReturnsBadRequest()
        {
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/set-step",
                new SetStepDto { TargetStep = 4 });

            Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        }

        [Fact]
        public async Task SetStep_TargetStepEqualsTotalSteps_Succeeds()
        {
            // Boundary/off-by-one guard: the controller rejects with strict `>`, so the last
            // valid step must NOT be rejected.
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/set-step",
                new SetStepDto { TargetStep = 3 });

            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        }

        [Fact]
        public async Task SetStep_NoActiveLesson_ReturnsBadRequest()
        {
            var f = await SeedAsync(totalSteps: 3);
            // Deliberately not starting a lesson.

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/set-step",
                new SetStepDto { TargetStep = 1 });

            Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        }

        // ---- Completed-but-still-controllable lifecycle (changed behavior) ------------------

        [Fact]
        public async Task UpdateProgress_StatusCompleted_KeepsActiveLessonIdSet()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress",
                new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var getResp = await f.Client.GetAsync($"api/RobotSession/{f.Session.Id}");
            var body = await getResp.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.NotNull(body!.ActiveLessonId);
            Assert.Equal(f.Lesson.Id, body.ActiveLessonId);
        }

        [Fact]
        public async Task UpdateProgress_StatusCompleted_ThenReplay_StillAccepted()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress",
                new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" });

            var replayResp = await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/replay", null);
            Assert.Equal(HttpStatusCode.OK, replayResp.StatusCode);
        }

        [Fact]
        public async Task UpdateProgress_StatusCompleted_ClosesLessonRunAsCompleted()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress",
                new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" });

            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var run = await db.LessonRuns.FirstAsync(r => r.RobotSessionId == f.Session.Id);
            Assert.Equal("completed", run.Status);
            Assert.NotNull(run.EndedAt);
        }

        [Fact]
        public async Task UpdateProgress_StatusCompleted_IsIdempotentOnDuplicateReports()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var dto = new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" };
            await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress", dto);

            using var scope1 = _factory.Services.CreateScope();
            var db1 = scope1.ServiceProvider.GetRequiredService<BloomDbContext>();
            var endedAtAfterFirst = (await db1.LessonRuns.FirstAsync(r => r.RobotSessionId == f.Session.Id)).EndedAt;

            var secondResp = await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress", dto);
            Assert.Equal(HttpStatusCode.OK, secondResp.StatusCode);

            using var scope2 = _factory.Services.CreateScope();
            var db2 = scope2.ServiceProvider.GetRequiredService<BloomDbContext>();
            var endedAtAfterSecond = (await db2.LessonRuns.FirstAsync(r => r.RobotSessionId == f.Session.Id)).EndedAt;

            Assert.Equal(endedAtAfterFirst, endedAtAfterSecond);
        }

        [Fact]
        public async Task UpdateProgress_StatusCompleted_MarksAssignmentCompletedExactlyOnce()
        {
            var f = await SeedAsync();

            // RobotSessionService's Completed branch looks up the Assignment by
            // (StudentId == session.UserId, LessonId == ActiveLessonId) -- keyed off the
            // session owner, not StartLessonDto.StudentId -- so the assignment here is seeded
            // against the SLP account that owns this fixture's session.
            using (var seedScope = _factory.Services.CreateScope())
            {
                var seedDb = seedScope.ServiceProvider.GetRequiredService<BloomDbContext>();
                TestDataSeeder.CreateAssignment(seedDb, studentId: f.Slp.Id, lessonId: f.Lesson.Id, assignedById: f.Slp.Id);
            }

            await StartLessonAsync(f);

            var dto = new UpdateLessonProgressDto { CurrentStepId = 3, CompletedSteps = 3, Status = "Completed" };
            await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress", dto);
            var secondResp = await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress", dto);
            Assert.Equal(HttpStatusCode.OK, secondResp.StatusCode); // no exception on the duplicate report

            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var assignment = await db.Assignments.FirstAsync(a => a.StudentId == f.Slp.Id && a.LessonId == f.Lesson.Id);
            Assert.True(assignment.IsCompleted);
        }

        [Fact]
        public async Task UpdateProgress_StatusInProgress_UpdatesProgressPercentage()
        {
            var f = await SeedAsync(totalSteps: 3);
            await StartLessonAsync(f);

            var resp = await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress",
                new UpdateLessonProgressDto { CurrentStepId = 1, CompletedSteps = 1, Status = "InProgress" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var progress = await db.LessonProgresses
                .FirstAsync(p => p.LessonId == f.Lesson.Id && p.StudentId == f.Slp.Id);
            Assert.Equal(33, progress.ProgressPercentage);
        }

        [Fact]
        public async Task UpdateProgress_NoActiveLesson_ReturnsInternalServerError()
        {
            // Documents CURRENT behavior, not necessarily correct behavior: UpdateLessonProgressAsync
            // throws InvalidOperationException when there's no active lesson, but the controller's
            // catch chain only names KeyNotFoundException/ArgumentException, so this falls through
            // to the generic handler and 500s instead of 400ing. Pinned here as a regression guard
            // and flagged as a candidate follow-up bug -- not fixed as part of this test pass.
            var f = await SeedAsync();

            var resp = await f.Client.PutAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/progress",
                new UpdateLessonProgressDto { CurrentStepId = 1, CompletedSteps = 1, Status = "InProgress" });

            Assert.Equal(HttpStatusCode.InternalServerError, resp.StatusCode);
        }

        // ---- repeat-request triggers auto-replay (new behavior) -----------------------------

        [Fact]
        public async Task RecordInteraction_RepeatPhrase_WithActiveLesson_QueuesReplayAndReturnsTrue()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/interactions",
                new LogLessonInteractionDto { StepId = 1, StudentResponse = "can you repeat that?" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var body = await resp.Content.ReadFromJsonAsync<System.Text.Json.JsonElement>(TestJson.Options);
            Assert.True(body.GetProperty("repeatRequested").GetBoolean());

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            Assert.Equal(HttpStatusCode.OK, poll.StatusCode);
            var control = await poll.Content.ReadFromJsonAsync<PendingStepControlDto>(TestJson.Options);
            Assert.Equal("repeat_last", control!.Command);
        }

        [Fact]
        public async Task RecordInteraction_RepeatPhrase_NoActiveLesson_ReturnsFalseAndDoesNotQueue()
        {
            var f = await SeedAsync();
            // Deliberately not starting a lesson.

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/interactions",
                new LogLessonInteractionDto { StepId = 1, StudentResponse = "can you repeat that?" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var body = await resp.Content.ReadFromJsonAsync<System.Text.Json.JsonElement>(TestJson.Options);
            Assert.False(body.GetProperty("repeatRequested").GetBoolean());

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            Assert.Equal(HttpStatusCode.NoContent, poll.StatusCode);
        }

        [Fact]
        public async Task RecordInteraction_OrdinaryResponse_DoesNotTriggerReplay()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/interactions",
                new LogLessonInteractionDto { StepId = 1, StudentResponse = "the cat" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var body = await resp.Content.ReadFromJsonAsync<System.Text.Json.JsonElement>(TestJson.Options);
            Assert.False(body.GetProperty("repeatRequested").GetBoolean());
        }

        [Fact]
        public async Task RecordInteraction_RepeatPhraseInRobotDialogue_DoesNotTriggerReplay()
        {
            // The robot logs its own spoken lines back to this same endpoint (see
            // LessonCoordinator::speak_script), with InteractionType "robot" and
            // the script text sitting in StudentResponse. A lesson script that
            // itself contains a phrase like "repeat that" (e.g. asking the
            // student to repeat something back) must not be misread as the
            // student requesting a repeat.
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/interactions",
                new LogLessonInteractionDto { StepId = 1, InteractionType = "robot", StudentResponse = "Can you repeat that back to me?" });
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var body = await resp.Content.ReadFromJsonAsync<System.Text.Json.JsonElement>(TestJson.Options);
            Assert.False(body.GetProperty("repeatRequested").GetBoolean());

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            Assert.Equal(HttpStatusCode.NoContent, poll.StatusCode);
        }

        [Fact]
        public async Task RecordInteraction_UnknownSession_ReturnsNotFound()
        {
            var f = await SeedAsync();

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{Guid.NewGuid()}/interactions",
                new LogLessonInteractionDto { StepId = 1, StudentResponse = "hello" });

            Assert.Equal(HttpStatusCode.NotFound, resp.StatusCode);
        }

        // ---- stale step-control cleanup on lesson restart (changed behavior) ----------------

        [Fact]
        public async Task StartLesson_ClearsStaleStepControlFromPriorRun()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);
            await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/skip", null);

            Lesson secondLesson;
            using (var scope = _factory.Services.CreateScope())
            {
                var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
                secondLesson = TestDataSeeder.CreateLessonWithSteps(db, f.Slp.Id, totalSteps: 2);
            }

            await StartLessonAsync(f, secondLesson.Id);

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            Assert.Equal(HttpStatusCode.NoContent, poll.StatusCode);

            using var verifyScope = _factory.Services.CreateScope();
            var verifyDb = verifyScope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var firstRun = await verifyDb.LessonRuns.FirstAsync(r => r.LessonId == f.Lesson.Id);
            var secondRun = await verifyDb.LessonRuns.FirstAsync(r => r.LessonId == secondLesson.Id);
            Assert.Equal("abandoned", firstRun.Status);
            Assert.Equal("active", secondRun.Status);
        }

        // ---- baseline smoke coverage ----------------------------------------------------------

        [Fact]
        public async Task StartLesson_QueuesLessonForRobotToPickUp()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var pending = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/pending-lesson");
            Assert.Equal(HttpStatusCode.OK, pending.StatusCode);
        }

        [Fact]
        public async Task StartLesson_AsNonOwner_ReturnsForbidden()
        {
            var f = await SeedAsync();
            using var scope = _factory.Services.CreateScope();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var intruder = await TestDataSeeder.CreateAccountAsync(users, role: "SLP");
            var intruderClient = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, intruder);

            var resp = await intruderClient.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/start",
                new StartLessonDto { LessonId = f.Lesson.Id, StudentId = intruder.Id });

            Assert.Equal(HttpStatusCode.Forbidden, resp.StatusCode);
        }

        [Fact]
        public async Task StopLesson_ClearsActiveLessonAndQueuesStopCommand()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/stop", null);
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var getResp = await f.Client.GetAsync($"api/RobotSession/{f.Session.Id}");
            var body = await getResp.Content.ReadFromJsonAsync<RobotSessionResponseDto>(TestJson.Options);
            Assert.Null(body!.ActiveLessonId);

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            Assert.Equal(HttpStatusCode.OK, poll.StatusCode);
            var control = await poll.Content.ReadFromJsonAsync<PendingStepControlDto>(TestJson.Options);
            Assert.Equal("stop", control!.Command);
        }

        [Fact]
        public async Task SkipStep_NoActiveLesson_ReturnsBadRequest()
        {
            var f = await SeedAsync();

            var resp = await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/skip", null);

            Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        }

        [Fact]
        public async Task PauseLesson_QueuesPauseCommand()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/pause", null);
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            var control = await poll.Content.ReadFromJsonAsync<PendingStepControlDto>(TestJson.Options);
            Assert.Equal("pause", control!.Command);
        }

        [Fact]
        public async Task ResumeLesson_QueuesResumeCommand()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/resume", null);
            Assert.Equal(HttpStatusCode.OK, resp.StatusCode);

            var poll = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/step-control");
            var control = await poll.Content.ReadFromJsonAsync<PendingStepControlDto>(TestJson.Options);
            Assert.Equal("resume", control!.Command);
        }

        [Fact]
        public async Task RecordFeedback_ApproveCommand_IsPolledAndAcknowledgeable()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var feedbackResp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/feedback",
                new RecordSLPFeedbackDto { StepId = 1, FeedbackCommand = "approve" });
            Assert.Equal(HttpStatusCode.OK, feedbackResp.StatusCode);

            var pollResp = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/pending-feedback");
            Assert.Equal(HttpStatusCode.OK, pollResp.StatusCode);
            var pending = await pollResp.Content.ReadFromJsonAsync<PendingFeedbackResponseDto>(TestJson.Options);
            Assert.True(pending!.HasPendingFeedback);
            Assert.Equal("approve", pending.FeedbackCommand);

            var ackResp = await f.Client.PutAsync(
                $"api/lesson-runtime/{f.Session.Id}/feedback/{pending.FeedbackId}/acknowledge", null);
            Assert.Equal(HttpStatusCode.OK, ackResp.StatusCode);

            var pollAgain = await f.Client.GetAsync($"api/lesson-runtime/{f.Session.Id}/pending-feedback");
            Assert.Equal(HttpStatusCode.NoContent, pollAgain.StatusCode);
        }

        [Fact]
        public async Task RecordFeedback_InvalidCommand_ReturnsBadRequest()
        {
            var f = await SeedAsync();
            await StartLessonAsync(f);

            var resp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/feedback",
                new RecordSLPFeedbackDto { StepId = 1, FeedbackCommand = "bogus" });

            Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        }

        [Fact]
        public async Task GetStudentLessonHistory_ReturnsRunsForStudent()
        {
            var f = await SeedAsync();
            using var scope = _factory.Services.CreateScope();
            var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var student = await TestDataSeeder.CreateAccountAsync(users, role: "Student");

            var startResp = await f.Client.PostAsJsonAsync($"api/lesson-runtime/{f.Session.Id}/start",
                new StartLessonDto { LessonId = f.Lesson.Id, StudentId = student.Id });
            Assert.Equal(HttpStatusCode.OK, startResp.StatusCode);
            await f.Client.PostAsync($"api/lesson-runtime/{f.Session.Id}/stop", null);

            var historyResp = await f.Client.GetAsync($"api/lesson-runtime/student/{student.Id}/history");
            Assert.Equal(HttpStatusCode.OK, historyResp.StatusCode);
            var history = await historyResp.Content
                .ReadFromJsonAsync<System.Collections.Generic.List<StudentLessonHistoryDto>>(TestJson.Options);
            Assert.NotNull(history);
            Assert.Contains(history!, h => h.LessonId == f.Lesson.Id);
        }
    }
}
