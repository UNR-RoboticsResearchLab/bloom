using System;
using System.Linq;
using System.Net;
using System.Net.Http.Json;
using System.Security.Claims;
using System.Text;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using bloom.Services.RemoteShell;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Auth rules and relaying for the remote shell's two SignalR hubs:
    /// /hubs/robot-agent (robot JWT, identity from the token only) and /hubs/shell (Admin cookie).
    /// The relay is a DI singleton, so every test uses its own freshly provisioned robot.
    /// </summary>
    public class RemoteShellRelayTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public RemoteShellRelayTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private IShellRelay Relay => _factory.Services.GetRequiredService<IShellRelay>();

        private async Task<(Robot Robot, string Secret)> ProvisionedRobotAsync()
        {
            using var scope = _factory.Services.CreateScope();
            return await TestDataSeeder.CreateProvisionedRobotAsync(
                scope.ServiceProvider.GetRequiredService<BloomDbContext>(),
                scope.ServiceProvider.GetRequiredService<IRobotService>());
        }

        private async Task<(Robot Robot, string Token)> RobotWithTokenAsync()
        {
            var (robot, secret) = await ProvisionedRobotAsync();
            return (robot, await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret));
        }

        private async Task<(Account Account, string Cookie)> AccountWithCookieAsync(bool admin = true)
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var account = admin
                ? await TestDataSeeder.CreateAdminAsync(userManager)
                : await TestDataSeeder.CreateAccountAsync(userManager, role: "SLP");
            return (account, await ShellTestClients.GetAuthCookieAsync(_factory, account));
        }

        private async Task<System.Net.Http.HttpClient> AdminHttpAsync(bool admin = true)
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var account = admin
                ? await TestDataSeeder.CreateAdminAsync(userManager)
                : await TestDataSeeder.CreateAccountAsync(userManager, role: "SLP");
            return await AuthTestHelper.CreateAuthorizedClientAsync(_factory, account);
        }

        private async Task WaitUntilOnlineAsync(Guid robotId, bool online = true)
        {
            for (var i = 0; i < 150 && Relay.IsOnline(robotId) != online; i++)
                await Task.Delay(20);
            Assert.Equal(online, Relay.IsOnline(robotId));
        }

        /// <summary>Connects an agent and an admin and returns both ends of an open session.</summary>
        private async Task<(Robot Robot, TestAgent Agent, TestAdmin Admin, Guid SessionId)> OpenSessionAsync()
        {
            var (robot, token) = await RobotWithTokenAsync();
            var agent = await TestAgent.ConnectAsync(_factory, token);
            await WaitUntilOnlineAsync(robot.Id);

            var (_, cookie) = await AccountWithCookieAsync();
            var admin = await TestAdmin.ConnectAsync(_factory, cookie);
            var opened = await admin.OpenSessionAsync(robot.Id);
            Assert.Equal("opened", opened.Status);
            var sessionId = opened.SessionId!.Value;

            var open = await agent.NextAsync("Open");
            Assert.Equal(sessionId, open.Arg<Guid>(0));
            Assert.Equal(100, open.Arg<int>(1));
            Assert.Equal(30, open.Arg<int>(2));
            return (robot, agent, admin, sessionId);
        }

        // ── Agent hub auth ─────────────────────────────────────────────────────

        [Fact]
        public async Task Agent_NoToken_IsRejected()
        {
            var status = await ShellTestClients.RefusedStatusAsync(() => TestAgent.ConnectAsync(_factory, token: ""));

            Assert.Equal(401, status);
        }

        [Fact]
        public async Task Agent_LegacyNameOnlyToken_IsRejected()
        {
            var robot = (await RobotWithTokenAsync()).Robot;
            var legacy = AuthTestHelper.MintJwt(_factory, new[]
            {
                new Claim("robotId", robot.Id.ToString()),
                new Claim(ClaimTypes.Role, "Robot"),
                new Claim(RobotService.CredentialClaim, "legacy"),
            });

            Assert.Equal(401, await ShellTestClients.RefusedStatusAsync(() => TestAgent.ConnectAsync(_factory, legacy)));
        }

        [Fact]
        public async Task Agent_StaleTokenVersion_IsRejected()
        {
            var robot = (await RobotWithTokenAsync()).Robot;
            var stale = AuthTestHelper.MintJwt(_factory, new[]
            {
                new Claim("robotId", robot.Id.ToString()),
                new Claim(ClaimTypes.Role, "Robot"),
                new Claim(RobotService.CredentialClaim, "secret"),
                new Claim(RobotService.TokenVersionClaim, "0"),  // provisioning already bumped it to 1
            });

            Assert.Equal(401, await ShellTestClients.RefusedStatusAsync(() => TestAgent.ConnectAsync(_factory, stale)));
        }

        [Fact]
        public async Task Agent_Hello_RecordsSemVerVersionAndCommit_ForTheTokensRobot()
        {
            var (robot, token) = await RobotWithTokenAsync();
            var (other, _) = await RobotWithTokenAsync();

            await using var agent = await TestAgent.ConnectAsync(_factory, token, hello: false);
            await agent.Hello("1.0.42-g1a2b3c4d", "1a2b3c4d");
            await WaitUntilOnlineAsync(robot.Id);

            Assert.False(Relay.IsOnline(other.Id));  // no method takes a robot id: identity is the token's
            using var scope = _factory.Services.CreateScope();
            var stored = await scope.ServiceProvider.GetRequiredService<BloomDbContext>()
                .Robots.AsNoTracking().FirstAsync(r => r.Id == robot.Id);
            Assert.Equal("1.0.42-g1a2b3c4d", stored.AgentVersion);
            Assert.Equal("1a2b3c4d", stored.AgentCommit);
            Assert.NotNull(stored.LastSeenAt);
        }

        [Fact]
        public async Task Agent_ConnectionIsClosedAtTokenExpiry_WithReason()
        {
            var robot = (await RobotWithTokenAsync()).Robot;
            var shortLived = AuthTestHelper.MintJwt(_factory, new[]
            {
                new Claim("robotId", robot.Id.ToString()),
                new Claim(ClaimTypes.Role, "Robot"),
                new Claim(RobotService.CredentialClaim, "secret"),
                new Claim(RobotService.TokenVersionClaim, "1"),
            }, expires: DateTime.UtcNow.AddSeconds(2));

            await using var agent = await TestAgent.ConnectAsync(_factory, shortLived);
            await WaitUntilOnlineAsync(robot.Id);

            var told = await agent.NextAsync("Disconnecting", TimeSpan.FromSeconds(10));
            Assert.Equal(ShellEndReason.TokenExpired, told.Arg<string>(0));
            Assert.True(await agent.WaitClosedAsync());
            await WaitUntilOnlineAsync(robot.Id, online: false);
        }

        // ── Admin hub auth ─────────────────────────────────────────────────────

        [Fact]
        public async Task Admin_NoCookie_IsRejected()
        {
            Assert.Equal(401, await ShellTestClients.RefusedStatusAsync(() => TestAdmin.ConnectAsync(_factory, cookie: null)));
        }

        [Fact]
        public async Task Admin_NonAdminCookie_IsForbidden()
        {
            var (_, cookie) = await AccountWithCookieAsync(admin: false);

            Assert.Equal(403, await ShellTestClients.RefusedStatusAsync(() => TestAdmin.ConnectAsync(_factory, cookie)));
        }

        [Fact]
        public async Task Admin_RobotJwt_CannotOpenShell()
        {
            var (_, token) = await RobotWithTokenAsync();

            Assert.Equal(401, await ShellTestClients.RefusedStatusAsync(
                () => TestAdmin.ConnectAsync(_factory, cookie: null, bearer: token)));
        }

        [Fact]
        public async Task Admin_CrossSiteOrigin_IsForbidden()
        {
            var (_, cookie) = await AccountWithCookieAsync();

            Assert.Equal(403, await ShellTestClients.RefusedStatusAsync(
                () => TestAdmin.ConnectAsync(_factory, cookie, origin: "https://evil.example")));
        }

        [Fact]
        public async Task Admin_OfflineRobot_IsReported()
        {
            var (robot, _) = await RobotWithTokenAsync();
            var (_, cookie) = await AccountWithCookieAsync();

            await using var admin = await TestAdmin.ConnectAsync(_factory, cookie);

            Assert.Equal("offline", (await admin.OpenSessionAsync(robot.Id)).Status);
            Assert.Equal("not_found", (await admin.OpenSessionAsync(Guid.NewGuid())).Status);
        }

        // ── Relaying ───────────────────────────────────────────────────────────

        [Fact]
        public async Task Session_RelaysInputOutputAndResize()
        {
            var (_, agent, admin, sessionId) = await OpenSessionAsync();
            await using (agent)
            await using (admin)
            {
                await admin.Input("ls\n");
                var input = await agent.NextAsync("Input");
                Assert.Equal(sessionId, input.Arg<Guid>(0));
                Assert.Equal("ls\n", Encoding.UTF8.GetString(input.Arg<byte[]>(1)));

                await agent.Output(sessionId, "file.txt\r\n");
                await admin.WaitForOutputAsync("file.txt\r\n");

                await admin.Resize(120, 40);
                var resize = await agent.NextAsync("Resize");
                Assert.Equal(sessionId, resize.Arg<Guid>(0));
                Assert.Equal(120, resize.Arg<int>(1));
                Assert.Equal(40, resize.Arg<int>(2));
            }
        }

        [Fact]
        public async Task Session_RobotCannotWriteIntoAnotherRobotsSession()
        {
            var (_, agent, admin, sessionId) = await OpenSessionAsync();
            var (other, otherToken) = await RobotWithTokenAsync();
            await using var otherAgent = await TestAgent.ConnectAsync(_factory, otherToken);
            await WaitUntilOnlineAsync(other.Id);
            await using (agent)
            await using (admin)
            {
                await otherAgent.Output(sessionId, "injected");
                await otherAgent.SessionExited(sessionId);  // nor can it end someone else's session
                await agent.Output(sessionId, "genuine");

                Assert.Equal("genuine", await admin.WaitForOutputAsync("genuine"));
                Assert.False(admin.SessionClosed.IsCompleted);
            }
        }

        [Fact]
        public async Task Session_ShellExit_ClosesAdminAndAudits()
        {
            var (robot, agent, admin, sessionId) = await OpenSessionAsync();
            await using (agent)
            await using (admin)
            {
                await agent.SessionExited(sessionId, 0);

                Assert.Equal(ShellEndReason.Exited, await admin.WaitSessionClosedAsync());
                await AssertAuditEndedAsync(sessionId, ShellEndReason.Exited);
                Assert.Null(Relay.GetActiveSession(robot.Id));
            }
        }

        [Fact]
        public async Task Session_AdminClose_TellsAgentAndAudits()
        {
            var (_, agent, admin, sessionId) = await OpenSessionAsync();
            await using (agent)
            await using (admin)
            {
                await admin.CloseSession();

                var close = await agent.NextAsync("Close");
                Assert.Equal(sessionId, close.Arg<Guid>(0));
                await AssertAuditEndedAsync(sessionId, ShellEndReason.AdminClosed);
            }
        }

        [Fact]
        public async Task Session_AdminDisappearing_EndsItAndTellsAgent()
        {
            var (_, agent, admin, sessionId) = await OpenSessionAsync();
            await using (agent)
            {
                await admin.DisposeAsync();  // browser tab closed

                var close = await agent.NextAsync("Close");
                Assert.Equal(sessionId, close.Arg<Guid>(0));
                await AssertAuditEndedAsync(sessionId, ShellEndReason.AdminDisconnected);
            }
        }

        [Fact]
        public async Task Session_SecondAdmin_IsBusy_UntilForcedTakeover()
        {
            var (robot, agent, first, firstSession) = await OpenSessionAsync();
            var (_, secondCookie) = await AccountWithCookieAsync();
            await using (agent)
            await using (first)
            {
                await using (var refused = await TestAdmin.ConnectAsync(_factory, secondCookie))
                {
                    var busy = await refused.OpenSessionAsync(robot.Id);
                    Assert.Equal("busy", busy.Status);
                    Assert.False(string.IsNullOrEmpty(busy.HeldBy));
                }

                await using var second = await TestAdmin.ConnectAsync(_factory, secondCookie);
                var taken = await second.OpenSessionAsync(robot.Id, force: true);
                Assert.Equal("opened", taken.Status);

                Assert.Equal(ShellEndReason.TakenOver, await first.WaitSessionClosedAsync());
                Assert.Equal(firstSession, (await agent.NextAsync("Close")).Arg<Guid>(0));
                Assert.Equal(taken.SessionId, (await agent.NextAsync("Open")).Arg<Guid>(0));
                await AssertAuditEndedAsync(firstSession, ShellEndReason.TakenOver);
            }
        }

        [Fact]
        public async Task Session_OneConnection_OneSession()
        {
            var (robot, agent, admin, _) = await OpenSessionAsync();
            await using (agent)
            await using (admin)
            {
                Assert.Equal("already_open", (await admin.OpenSessionAsync(robot.Id)).Status);
            }
        }

        [Fact]
        public async Task Revoke_DropsLiveConnectionAndSession()
        {
            var (robot, agent, admin, sessionId) = await OpenSessionAsync();
            var http = await AdminHttpAsync();
            await using (agent)
            await using (admin)
            {
                (await http.PostAsync($"api/robot/{robot.Id}/revoke", null)).EnsureSuccessStatusCode();

                Assert.Equal(ShellEndReason.Revoked, (await agent.NextAsync("Disconnecting")).Arg<string>(0));
                Assert.Equal(ShellEndReason.Revoked, await admin.WaitSessionClosedAsync());
                Assert.True(await agent.WaitClosedAsync());
                await WaitUntilOnlineAsync(robot.Id, online: false);
                await AssertAuditEndedAsync(sessionId, ShellEndReason.Revoked);
            }
        }

        [Fact]
        public async Task OpeningSessions_IsRateLimitedPerAdmin()
        {
            var (robot, _) = await RobotWithTokenAsync();  // offline: every attempt still counts
            var (_, cookie) = await AccountWithCookieAsync();
            await using var admin = await TestAdmin.ConnectAsync(_factory, cookie);

            string? last = null;
            for (var i = 0; i < 6; i++)
                last = (await admin.OpenSessionAsync(robot.Id)).Status;

            Assert.Equal("rate_limited", last);
        }

        [Fact]
        public async Task Fleet_ShowsOnlineVersionAndShellHolder()
        {
            var (robot, agent, admin, _) = await OpenSessionAsync();
            var http = await AdminHttpAsync();
            await using (agent)
            await using (admin)
            {
                var fleet = await http.GetFromJsonAsync<RobotFleetDto[]>("api/robot/fleet", TestJson.Options);

                var entry = fleet!.Single(r => r.Id == robot.Id);
                Assert.True(entry.Online);
                Assert.True(entry.Provisioned);
                Assert.Equal("1.0.7-g1a2b3c4d", entry.AgentVersion);
                Assert.NotNull(entry.ShellHeldBy);
            }
        }

        // ── Credential push ────────────────────────────────────────────────────

        private async Task<(Robot Robot, string Secret, TestAgent Agent)> OnlineProvisionedRobotAsync()
        {
            var (robot, secret) = await ProvisionedRobotAsync();
            var agent = await TestAgent.ConnectAsync(_factory, await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret));
            await WaitUntilOnlineAsync(robot.Id);
            return (robot, secret, agent);
        }

        private async Task<HttpStatusCode> LoginStatusAsync(Guid robotId, string secret) =>
            (await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robotId, Secret = secret })).StatusCode;

        [Fact]
        public async Task Push_Applied_DeliversSecret_AndRetiresTheOldOneOnFirstUse()
        {
            var (robot, oldSecret, agent) = await OnlineProvisionedRobotAsync();
            var admin = await AdminHttpAsync();
            await using (agent)
            {
                var post = admin.PostAsync($"api/robot/{robot.Id}/credentials/push", null);

                var push = await agent.NextAsync("SetSecret");
                var requestId = push.Arg<Guid>(0);
                var newSecret = push.Arg<string>(1);
                Assert.NotEqual(oldSecret, newSecret);
                await agent.SecretApplied(requestId, ok: true);

                var response = await post;
                Assert.Equal(HttpStatusCode.OK, response.StatusCode);
                Assert.DoesNotContain(newSecret, await response.Content.ReadAsStringAsync());

                // The connection is dropped so the agent logs in again -- with the new secret.
                Assert.Equal(ShellEndReason.CredentialsRotated, (await agent.NextAsync("Disconnecting")).Arg<string>(0));
                Assert.True(await agent.WaitClosedAsync());
                await WaitUntilOnlineAsync(robot.Id, online: false);

                Assert.Equal(HttpStatusCode.OK, await LoginStatusAsync(robot.Id, oldSecret));    // not retired yet
                Assert.Equal(HttpStatusCode.OK, await LoginStatusAsync(robot.Id, newSecret));    // promotes
                Assert.Equal(HttpStatusCode.Unauthorized, await LoginStatusAsync(robot.Id, oldSecret));
            }
        }

        [Fact]
        public async Task Push_Rejected_ChangesNothing()
        {
            var (robot, oldSecret, agent) = await OnlineProvisionedRobotAsync();
            var admin = await AdminHttpAsync();
            await using (agent)
            {
                var post = admin.PostAsync($"api/robot/{robot.Id}/credentials/push", null);
                var push = await agent.NextAsync("SetSecret");
                await agent.SecretApplied(push.Arg<Guid>(0), ok: false, "cannot write /etc/bloom-agent/secret");

                var response = await post;
                Assert.Equal(HttpStatusCode.BadGateway, response.StatusCode);
                Assert.Contains("cannot write", await response.Content.ReadAsStringAsync());
                Assert.True(Relay.IsOnline(robot.Id));  // still connected; nothing was rotated

                Assert.Equal(HttpStatusCode.OK, await LoginStatusAsync(robot.Id, oldSecret));
                Assert.Equal(HttpStatusCode.Unauthorized, await LoginStatusAsync(robot.Id, push.Arg<string>(1)));  // pending discarded
            }
        }

        [Fact]
        public async Task Push_Unconfirmed_KeepsBothSecretsWorking()
        {
            var (robot, oldSecret, agent) = await OnlineProvisionedRobotAsync();
            var admin = await AdminHttpAsync();
            await using (agent)
            {
                var post = admin.PostAsync($"api/robot/{robot.Id}/credentials/push", null);
                var push = await agent.NextAsync("SetSecret");   // ...and never answer

                Assert.Equal(HttpStatusCode.GatewayTimeout, (await post).StatusCode);
                Assert.Equal(HttpStatusCode.OK, await LoginStatusAsync(robot.Id, oldSecret));
                Assert.Equal(HttpStatusCode.OK, await LoginStatusAsync(robot.Id, push.Arg<string>(1)));
            }
        }

        [Fact]
        public async Task Push_AckFromAnotherRobot_IsIgnored()
        {
            var (robot, _, agent) = await OnlineProvisionedRobotAsync();
            var (_, _, otherAgent) = await OnlineProvisionedRobotAsync();
            var admin = await AdminHttpAsync();
            await using (agent)
            await using (otherAgent)
            {
                var post = admin.PostAsync($"api/robot/{robot.Id}/credentials/push", null);
                var requestId = (await agent.NextAsync("SetSecret")).Arg<Guid>(0);

                await otherAgent.SecretApplied(requestId, ok: true);  // not its push
                await agent.SecretApplied(requestId, ok: false, "no");

                Assert.Equal(HttpStatusCode.BadGateway, (await post).StatusCode);
            }
        }

        [Fact]
        public async Task Push_OfflineRobot_IsConflict_AndNonAdminIsForbidden()
        {
            var (robot, _) = await ProvisionedRobotAsync();

            Assert.Equal(HttpStatusCode.Conflict,
                (await (await AdminHttpAsync()).PostAsync($"api/robot/{robot.Id}/credentials/push", null)).StatusCode);
            Assert.Equal(HttpStatusCode.Forbidden,
                (await (await AdminHttpAsync(admin: false)).PostAsync($"api/robot/{robot.Id}/credentials/push", null)).StatusCode);
            Assert.Equal(HttpStatusCode.Unauthorized,
                (await _factory.CreateClient().PostAsync($"api/robot/{robot.Id}/credentials/push", null)).StatusCode);
        }

        [Fact]
        public async Task Delete_DropsLiveConnectionAndSession_ButKeepsAuditLog()
        {
            var (robot, agent, admin, sessionId) = await OpenSessionAsync();
            var http = await AdminHttpAsync();
            await using (agent)
            await using (admin)
            {
                Assert.Equal(HttpStatusCode.OK, (await http.DeleteAsync($"api/robot/{robot.Id}")).StatusCode);

                Assert.Equal(ShellEndReason.RobotDeleted, (await agent.NextAsync("Disconnecting")).Arg<string>(0));
                Assert.Equal(ShellEndReason.RobotDeleted, await admin.WaitSessionClosedAsync());
                await WaitUntilOnlineAsync(robot.Id, online: false);
                await AssertAuditEndedAsync(sessionId, ShellEndReason.RobotDeleted);  // history outlives the robot
            }
        }

        private async Task AssertAuditEndedAsync(Guid sessionId, string reason)
        {
            ShellSessionAudit? audit = null;
            for (var i = 0; i < 150; i++)
            {
                using var scope = _factory.Services.CreateScope();
                audit = await scope.ServiceProvider.GetRequiredService<BloomDbContext>()
                    .ShellSessionAudits.AsNoTracking().FirstOrDefaultAsync(a => a.Id == sessionId);
                if (audit?.EndedAt != null)
                    break;
                await Task.Delay(20);
            }
            Assert.NotNull(audit);
            Assert.Equal(reason, audit!.EndReason);
            Assert.NotNull(audit.EndedAt);
        }
    }
}
