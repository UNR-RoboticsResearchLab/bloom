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
