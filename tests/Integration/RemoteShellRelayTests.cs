using System;
using System.Linq;
using System.Net.Http.Json;
using System.Net.WebSockets;
using System.Security.Claims;
using System.Text;
using System.Text.Json;
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
    /// Auth rules and relaying for the remote shell's two WebSocket endpoints:
    /// /ws/robot-agent (robot JWT, identity from the token only) and /ws/shell (Admin cookie).
    /// The relay is a DI singleton, so every test uses its own freshly provisioned robot.
    /// </summary>
    public class RemoteShellRelayTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public RemoteShellRelayTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private async Task<(Robot Robot, string Token)> RobotWithTokenAsync()
        {
            using var scope = _factory.Services.CreateScope();
            var (robot, secret) = await TestDataSeeder.CreateProvisionedRobotAsync(
                scope.ServiceProvider.GetRequiredService<BloomDbContext>(),
                scope.ServiceProvider.GetRequiredService<IRobotService>());
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

        private IShellRelay Relay => _factory.Services.GetRequiredService<IShellRelay>();

        private async Task WaitUntilOnlineAsync(Guid robotId, bool online = true)
        {
            for (var i = 0; i < 100 && Relay.IsOnline(robotId) != online; i++)
                await Task.Delay(20);
            Assert.Equal(online, Relay.IsOnline(robotId));
        }

        /// <summary>Connects an agent and an admin and returns both ends of an open session.</summary>
        private async Task<(Robot Robot, TestShellSocket Agent, TestShellSocket Admin, Guid SessionId)> OpenSessionAsync()
        {
            var (robot, token) = await RobotWithTokenAsync();
            var agent = await ShellTestClients.ConnectAgentAsync(_factory, token);
            await WaitUntilOnlineAsync(robot.Id);

            var (_, cookie) = await AccountWithCookieAsync();
            var admin = await ShellTestClients.ConnectAdminAsync(_factory, cookie, robot.Id);
            var opened = await admin.ReceiveJsonAsync();
            Assert.Equal("opened", opened.GetProperty("type").GetString());
            var sessionId = opened.GetProperty("sessionId").GetGuid();

            var open = await agent.ReceiveJsonAsync();
            Assert.Equal("open", open.GetProperty("type").GetString());
            Assert.Equal(sessionId, open.GetProperty("sessionId").GetGuid());
            Assert.Equal(100, open.GetProperty("cols").GetInt32());
            Assert.Equal(30, open.GetProperty("rows").GetInt32());

            return (robot, agent, admin, sessionId);
        }

        // ── Agent endpoint auth ────────────────────────────────────────────────

        [Fact]
        public async Task Agent_NoToken_IsRejected()
        {
            var status = await ShellTestClients.HandshakeStatusAsync(async () =>
            {
                var client = _factory.Server.CreateWebSocketClient();
                await client.ConnectAsync(new Uri("ws://localhost/ws/robot-agent"), default);
            });

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

            var status = await ShellTestClients.HandshakeStatusAsync(() => ShellTestClients.ConnectAgentAsync(_factory, legacy));

            Assert.Equal(401, status);
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

            var status = await ShellTestClients.HandshakeStatusAsync(() => ShellTestClients.ConnectAgentAsync(_factory, stale));

            Assert.Equal(401, status);
        }

        [Fact]
        public async Task Agent_IdentityComesFromToken_NotFromMessages()
        {
            var (robot, token) = await RobotWithTokenAsync();
            var (victim, _) = await RobotWithTokenAsync();

            await using var agent = await ShellTestClients.ConnectAgentAsync(_factory, token,
                new { type = "hello", version = "1.0.7", commit = "abc", robotId = victim.Id });
            await WaitUntilOnlineAsync(robot.Id);

            Assert.False(Relay.IsOnline(victim.Id));
        }

        [Fact]
        public async Task Agent_Hello_RecordsSemVerVersionAndCommit()
        {
            var (robot, token) = await RobotWithTokenAsync();

            await using var agent = await ShellTestClients.ConnectAgentAsync(_factory, token,
                new { type = "hello", version = "1.0.42-g1a2b3c4d", commit = "1a2b3c4d" });
            await WaitUntilOnlineAsync(robot.Id);

            using var scope = _factory.Services.CreateScope();
            var stored = await scope.ServiceProvider.GetRequiredService<BloomDbContext>()
                .Robots.AsNoTracking().FirstAsync(r => r.Id == robot.Id);
            Assert.Equal("1.0.42-g1a2b3c4d", stored.AgentVersion);
            Assert.Equal("1a2b3c4d", stored.AgentCommit);
            Assert.NotNull(stored.LastSeenAt);
        }

        [Fact]
        public async Task Agent_TunnelIsClosedAtTokenExpiry_WithReason()
        {
            var robot = (await RobotWithTokenAsync()).Robot;
            // JwtBearer's default 5 minute clock skew still accepts this; the relay closes at "exp".
            var shortLived = AuthTestHelper.MintJwt(_factory, new[]
            {
                new Claim("robotId", robot.Id.ToString()),
                new Claim(ClaimTypes.Role, "Robot"),
                new Claim(RobotService.CredentialClaim, "secret"),
                new Claim(RobotService.TokenVersionClaim, "1"),
            }, expires: DateTime.UtcNow.AddSeconds(2));

            await using var agent = await ShellTestClients.ConnectAgentAsync(_factory, shortLived);
            await WaitUntilOnlineAsync(robot.Id);

            Assert.Null(await agent.ReceiveUntilClosedAsync(TimeSpan.FromSeconds(10)));
            Assert.Equal(WebSocketCloseStatus.PolicyViolation, agent.CloseStatus);
            Assert.Equal(ShellEndReason.TokenExpired, agent.CloseStatusDescription);
            // Reply to the close like the real agent (Beast does it automatically); the server's
            // receive loop ends on that reply rather than an abort.
            await agent.DisposeAsync();
            await WaitUntilOnlineAsync(robot.Id, online: false);
        }

        // ── Admin endpoint auth ────────────────────────────────────────────────

        [Fact]
        public async Task Admin_NoCookie_IsRejected()
        {
            var status = await ShellTestClients.HandshakeStatusAsync(
                () => ShellTestClients.ConnectAdminAsync(_factory, cookie: null, Guid.NewGuid()));

            Assert.Equal(401, status);
        }

        [Fact]
        public async Task Admin_NonAdminCookie_IsForbidden()
        {
            var (_, cookie) = await AccountWithCookieAsync(admin: false);

            var status = await ShellTestClients.HandshakeStatusAsync(
                () => ShellTestClients.ConnectAdminAsync(_factory, cookie, Guid.NewGuid()));

            Assert.Equal(403, status);
        }

        [Fact]
        public async Task Admin_RobotJwt_CannotOpenShell()
        {
            var (robot, token) = await RobotWithTokenAsync();

            var status = await ShellTestClients.HandshakeStatusAsync(
                () => ShellTestClients.ConnectAdminAsync(_factory, cookie: null, robot.Id, bearer: token));

            Assert.Equal(401, status);
        }

        [Fact]
        public async Task Admin_CrossSiteOrigin_IsForbidden()
        {
            var (_, cookie) = await AccountWithCookieAsync();

            var status = await ShellTestClients.HandshakeStatusAsync(
                () => ShellTestClients.ConnectAdminAsync(_factory, cookie, Guid.NewGuid(), origin: "https://evil.example"));

            Assert.Equal(403, status);
        }

        [Fact]
        public async Task Admin_OfflineRobot_GetsError()
        {
            var (robot, _) = await RobotWithTokenAsync();
            var (_, cookie) = await AccountWithCookieAsync();

            await using var admin = await ShellTestClients.ConnectAdminAsync(_factory, cookie, robot.Id);
            var reply = await admin.ReceiveJsonAsync();

            Assert.Equal("error", reply.GetProperty("type").GetString());
        }

        // ── Relaying ───────────────────────────────────────────────────────────

        [Fact]
        public async Task Session_RelaysInputOutputAndResize()
        {
            var (_, agent, admin, sessionId) = await OpenSessionAsync();
            await using (agent)
            await using (admin)
            {
                await admin.SendBinaryAsync(Encoding.UTF8.GetBytes("ls\n"));
                var input = await agent.ReceiveAsync();
                Assert.Equal(WebSocketMessageType.Binary, input!.Value.Type);
                Assert.Equal(ShellTestClients.DataFrame(sessionId, "ls\n"), input.Value.Data);

                await agent.SendBinaryAsync(ShellTestClients.DataFrame(sessionId, "file.txt\r\n"));
                var output = await admin.ReceiveAsync();
                Assert.Equal(WebSocketMessageType.Binary, output!.Value.Type);
                Assert.Equal("file.txt\r\n", Encoding.UTF8.GetString(output.Value.Data));

                await admin.SendJsonAsync(new { type = "resize", cols = 120, rows = 40 });
                var resize = await agent.ReceiveJsonAsync();
                Assert.Equal("resize", resize.GetProperty("type").GetString());
                Assert.Equal(sessionId, resize.GetProperty("sessionId").GetGuid());
                Assert.Equal(120, resize.GetProperty("cols").GetInt32());
            }
        }

        [Fact]
        public async Task Session_RobotCannotWriteIntoAnotherRobotsSession()
        {
            var (_, agent, admin, sessionId) = await OpenSessionAsync();
            var (other, otherToken) = await RobotWithTokenAsync();
            await using var otherAgent = await ShellTestClients.ConnectAgentAsync(_factory, otherToken);
            await WaitUntilOnlineAsync(other.Id);
            await using (agent)
            await using (admin)
            {
                await otherAgent.SendBinaryAsync(ShellTestClients.DataFrame(sessionId, "injected"));
                await agent.SendBinaryAsync(ShellTestClients.DataFrame(sessionId, "genuine"));

                var output = await admin.ReceiveAsync();
                Assert.Equal("genuine", Encoding.UTF8.GetString(output!.Value.Data));
            }
        }

        [Fact]
        public async Task Session_ShellExit_ClosesAdminAndAudits()
        {
            var (robot, agent, admin, sessionId) = await OpenSessionAsync();
            await using (agent)
            await using (admin)
            {
                await agent.SendJsonAsync(new { type = "exited", sessionId, code = 0 });

                var last = await admin.ReceiveUntilClosedAsync();
                Assert.Equal("exited", last!.Value.GetProperty("reason").GetString());
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
                await admin.SendJsonAsync(new { type = "close" });

                var close = await agent.ReceiveJsonAsync();
                Assert.Equal("close", close.GetProperty("type").GetString());
                Assert.Equal(sessionId, close.GetProperty("sessionId").GetGuid());
                await admin.ReceiveUntilClosedAsync();
                await AssertAuditEndedAsync(sessionId, ShellEndReason.AdminClosed);
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
                await using (var refused = await ShellTestClients.ConnectAdminAsync(_factory, secondCookie, robot.Id))
                {
                    var busy = await refused.ReceiveJsonAsync();
                    Assert.Equal("busy", busy.GetProperty("type").GetString());
                    Assert.False(string.IsNullOrEmpty(busy.GetProperty("heldBy").GetString()));
                }

                await using var second = await ShellTestClients.ConnectAdminAsync(_factory, secondCookie, robot.Id, force: true);
                Assert.Equal("opened", (await second.ReceiveJsonAsync()).GetProperty("type").GetString());

                var last = await first.ReceiveUntilClosedAsync();
                Assert.Equal(ShellEndReason.TakenOver, last!.Value.GetProperty("reason").GetString());
                await AssertAuditEndedAsync(firstSession, ShellEndReason.TakenOver);
            }
        }

        [Fact]
        public async Task Revoke_DropsLiveTunnelAndSession()
        {
            var (robot, agent, admin, sessionId) = await OpenSessionAsync();
            using var scope = _factory.Services.CreateScope();
            var adminAccount = await TestDataSeeder.CreateAdminAsync(scope.ServiceProvider.GetRequiredService<UserManager<Account>>());
            var http = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, adminAccount);
            await using (agent)
            await using (admin)
            {
                var response = await http.PostAsync($"api/robot/{robot.Id}/revoke", null);
                response.EnsureSuccessStatusCode();

                Assert.Null(await agent.ReceiveUntilClosedAsync());
                var last = await admin.ReceiveUntilClosedAsync();
                Assert.Equal(ShellEndReason.Revoked, last!.Value.GetProperty("reason").GetString());
                await WaitUntilOnlineAsync(robot.Id, online: false);
                await AssertAuditEndedAsync(sessionId, ShellEndReason.Revoked);
            }
        }

        [Fact]
        public async Task OpeningSessions_IsRateLimitedPerAdmin()
        {
            var (robot, _) = await RobotWithTokenAsync();  // offline: every attempt still counts
            var (_, cookie) = await AccountWithCookieAsync();

            string? lastMessage = null;
            for (var i = 0; i < 6; i++)
            {
                await using var admin = await ShellTestClients.ConnectAdminAsync(_factory, cookie, robot.Id);
                lastMessage = (await admin.ReceiveJsonAsync()).GetProperty("message").GetString();
            }

            Assert.Contains("Too many", lastMessage);
        }

        [Fact]
        public async Task Fleet_ShowsOnlineVersionAndShellHolder()
        {
            var (robot, agent, admin, _) = await OpenSessionAsync();
            using var scope = _factory.Services.CreateScope();
            var adminAccount = await TestDataSeeder.CreateAdminAsync(scope.ServiceProvider.GetRequiredService<UserManager<Account>>());
            var http = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, adminAccount);
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

        private async Task<System.Net.Http.HttpClient> AdminHttpAsync(bool admin = true)
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var account = admin
                ? await TestDataSeeder.CreateAdminAsync(userManager)
                : await TestDataSeeder.CreateAccountAsync(userManager, role: "SLP");
            return await AuthTestHelper.CreateAuthorizedClientAsync(_factory, account);
        }

        private async Task<(Robot Robot, string Secret, TestShellSocket Agent)> OnlineProvisionedRobotAsync()
        {
            using var scope = _factory.Services.CreateScope();
            var (robot, secret) = await TestDataSeeder.CreateProvisionedRobotAsync(
                scope.ServiceProvider.GetRequiredService<BloomDbContext>(),
                scope.ServiceProvider.GetRequiredService<IRobotService>());
            var agent = await ShellTestClients.ConnectAgentAsync(_factory,
                await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret));
            await WaitUntilOnlineAsync(robot.Id);
            return (robot, secret, agent);
        }

        private async Task<System.Net.Http.HttpResponseMessage> LoginAsync(Guid robotId, string secret) =>
            await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robotId, Secret = secret });

        [Fact]
        public async Task Push_Applied_DeliversSecret_AndRetiresTheOldOneOnFirstUse()
        {
            var (robot, oldSecret, agent) = await OnlineProvisionedRobotAsync();
            var admin = await AdminHttpAsync();
            await using (agent)
            {
                var post = admin.PostAsync($"api/robot/{robot.Id}/credentials/push", null);

                var push = await agent.ReceiveJsonAsync();
                Assert.Equal("set_secret", push.GetProperty("type").GetString());
                var newSecret = push.GetProperty("secret").GetString()!;
                Assert.NotEqual(oldSecret, newSecret);
                await agent.SendJsonAsync(new { type = "secret_applied", requestId = push.GetProperty("requestId").GetGuid(), ok = true });

                var response = await post;
                Assert.Equal(System.Net.HttpStatusCode.OK, response.StatusCode);
                Assert.DoesNotContain(newSecret, await response.Content.ReadAsStringAsync());

                // The tunnel is dropped so the agent logs in again -- with the new secret.
                Assert.Null(await agent.ReceiveUntilClosedAsync());
                Assert.Equal(ShellEndReason.CredentialsRotated, agent.CloseStatusDescription);
                await WaitUntilOnlineAsync(robot.Id, online: false);

                Assert.Equal(System.Net.HttpStatusCode.OK, (await LoginAsync(robot.Id, oldSecret)).StatusCode);  // not retired yet
                Assert.Equal(System.Net.HttpStatusCode.OK, (await LoginAsync(robot.Id, newSecret)).StatusCode);  // promotes
                Assert.Equal(System.Net.HttpStatusCode.Unauthorized, (await LoginAsync(robot.Id, oldSecret)).StatusCode);
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
                var push = await agent.ReceiveJsonAsync();
                await agent.SendJsonAsync(new { type = "secret_applied", requestId = push.GetProperty("requestId").GetGuid(), ok = false, message = "cannot write /etc/bloom-agent/secret" });

                var response = await post;
                Assert.Equal(System.Net.HttpStatusCode.BadGateway, response.StatusCode);
                Assert.Contains("cannot write", await response.Content.ReadAsStringAsync());
                Assert.True(Relay.IsOnline(robot.Id));  // still connected; nothing was rotated

                Assert.Equal(System.Net.HttpStatusCode.OK, (await LoginAsync(robot.Id, oldSecret)).StatusCode);
                Assert.Equal(System.Net.HttpStatusCode.Unauthorized,
                    (await LoginAsync(robot.Id, push.GetProperty("secret").GetString()!)).StatusCode);  // pending was discarded
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
                var push = await agent.ReceiveJsonAsync();   // ...and never answer

                Assert.Equal(System.Net.HttpStatusCode.GatewayTimeout, (await post).StatusCode);
                Assert.Equal(System.Net.HttpStatusCode.OK, (await LoginAsync(robot.Id, oldSecret)).StatusCode);
                Assert.Equal(System.Net.HttpStatusCode.OK, (await LoginAsync(robot.Id, push.GetProperty("secret").GetString()!)).StatusCode);
            }
        }

        [Fact]
        public async Task Push_AckFromAnotherRobot_IsIgnored()
        {
            var (robot, _, agent) = await OnlineProvisionedRobotAsync();
            var (other, _, otherAgent) = await OnlineProvisionedRobotAsync();
            var admin = await AdminHttpAsync();
            await using (agent)
            await using (otherAgent)
            {
                var post = admin.PostAsync($"api/robot/{robot.Id}/credentials/push", null);
                var push = await agent.ReceiveJsonAsync();
                var requestId = push.GetProperty("requestId").GetGuid();

                await otherAgent.SendJsonAsync(new { type = "secret_applied", requestId, ok = true });  // not its push
                await agent.SendJsonAsync(new { type = "secret_applied", requestId, ok = false, message = "no" });

                Assert.Equal(System.Net.HttpStatusCode.BadGateway, (await post).StatusCode);
            }
        }

        [Fact]
        public async Task Push_OfflineRobot_IsConflict_AndNonAdminIsForbidden()
        {
            using var scope = _factory.Services.CreateScope();
            var (robot, _) = await TestDataSeeder.CreateProvisionedRobotAsync(
                scope.ServiceProvider.GetRequiredService<BloomDbContext>(),
                scope.ServiceProvider.GetRequiredService<IRobotService>());

            Assert.Equal(System.Net.HttpStatusCode.Conflict,
                (await (await AdminHttpAsync()).PostAsync($"api/robot/{robot.Id}/credentials/push", null)).StatusCode);
            Assert.Equal(System.Net.HttpStatusCode.Forbidden,
                (await (await AdminHttpAsync(admin: false)).PostAsync($"api/robot/{robot.Id}/credentials/push", null)).StatusCode);
            Assert.Equal(System.Net.HttpStatusCode.Unauthorized,
                (await _factory.CreateClient().PostAsync($"api/robot/{robot.Id}/credentials/push", null)).StatusCode);
        }

        [Fact]
        public async Task Delete_DropsLiveTunnelAndSession_ButKeepsAuditLog()
        {
            var (robot, agent, admin, sessionId) = await OpenSessionAsync();
            var http = await AdminHttpAsync();
            await using (agent)
            await using (admin)
            {
                Assert.Equal(System.Net.HttpStatusCode.OK, (await http.DeleteAsync($"api/robot/{robot.Id}")).StatusCode);

                Assert.Null(await agent.ReceiveUntilClosedAsync());
                var last = await admin.ReceiveUntilClosedAsync();
                Assert.Equal(ShellEndReason.RobotDeleted, last!.Value.GetProperty("reason").GetString());
                await WaitUntilOnlineAsync(robot.Id, online: false);
                await AssertAuditEndedAsync(sessionId, ShellEndReason.RobotDeleted);  // history outlives the robot
            }
        }

        private async Task AssertAuditEndedAsync(Guid sessionId, string reason)
        {
            ShellSessionAudit? audit = null;
            for (var i = 0; i < 100; i++)
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
