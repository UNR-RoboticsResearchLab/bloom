using System;
using System.Net;
using System.Net.Http.Json;
using System.Security.Claims;
using System.Text.Json;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Services;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Robot credentials: admin-only provisioning, RobotId + Secret login, and the TokenVersion
    /// check that makes rotate/revoke take effect before a token expires.
    /// </summary>
    public class RobotAuthTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public RobotAuthTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private async Task<(Robot Robot, string Secret)> ProvisionRobotAsync()
        {
            using var scope = _factory.Services.CreateScope();
            var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
            var robots = scope.ServiceProvider.GetRequiredService<IRobotService>();
            return await TestDataSeeder.CreateProvisionedRobotAsync(db, robots);
        }

        private async Task<System.Net.Http.HttpClient> AdminClientAsync()
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var admin = await TestDataSeeder.CreateAdminAsync(userManager);
            return await AuthTestHelper.CreateAuthorizedClientAsync(_factory, admin);
        }

        private static object NewRobotDto() =>
            new { Name = $"Provisioned {Guid.NewGuid():N}", Model = "Blossom", IPAddress = "N/A" };

        [Fact]
        public async Task Register_Anonymous_ReturnsUnauthorized()
        {
            var response = await _factory.CreateClient().PostAsJsonAsync("api/robot/register", NewRobotDto());

            Assert.Equal(HttpStatusCode.Unauthorized, response.StatusCode);
        }

        [Fact]
        public async Task Register_AsAdmin_ReturnsSecretThatLogsIn()
        {
            var admin = await AdminClientAsync();

            var response = await admin.PostAsJsonAsync("api/robot/register", NewRobotDto());

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
            var robotId = json.RootElement.GetProperty("robot").GetProperty("id").GetGuid();
            var secret = json.RootElement.GetProperty("secret").GetString()!;
            Assert.False(json.RootElement.GetProperty("robot").TryGetProperty("secretHash", out _));

            var token = await AuthTestHelper.LoginRobotAsync(_factory, robotId, secret);
            Assert.False(string.IsNullOrEmpty(token));
        }

        [Fact]
        public async Task Login_WrongSecret_ReturnsUnauthorized()
        {
            var (robot, _) = await ProvisionRobotAsync();

            var response = await _factory.CreateClient().PostAsJsonAsync("api/robot/login",
                new { RobotId = robot.Id, Secret = "not-the-secret" });

            Assert.Equal(HttpStatusCode.Unauthorized, response.StatusCode);
        }

        [Fact]
        public async Task Login_NameOnly_ReturnsUnauthorized_WhenLegacyLoginDisabled()
        {
            var (robot, _) = await ProvisionRobotAsync();

            var response = await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { robot.Name });

            Assert.Equal(HttpStatusCode.Unauthorized, response.StatusCode);
        }

        [Fact]
        public async Task Login_ReturnsRobotId()
        {
            var (robot, secret) = await ProvisionRobotAsync();

            var response = await _factory.CreateClient().PostAsJsonAsync("api/robot/login",
                new { RobotId = robot.Id, Secret = secret });

            using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
            Assert.Equal(robot.Id, json.RootElement.GetProperty("robotId").GetGuid());
        }

        [Fact]
        public async Task RotateCredentials_InvalidatesOldTokenAndSecret()
        {
            var (robot, secret) = await ProvisionRobotAsync();
            var oldToken = await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret);
            var robotClient = AuthTestHelper.CreateBearerClient(_factory, oldToken);
            Assert.Equal(HttpStatusCode.OK, (await robotClient.GetAsync($"api/robot/{robot.Id}")).StatusCode);

            var admin = await AdminClientAsync();
            var rotate = await admin.PostAsync($"api/robot/{robot.Id}/credentials", null);
            Assert.Equal(HttpStatusCode.OK, rotate.StatusCode);
            using var json = JsonDocument.Parse(await rotate.Content.ReadAsStringAsync());
            var newSecret = json.RootElement.GetProperty("secret").GetString()!;

            Assert.Equal(HttpStatusCode.Unauthorized, (await robotClient.GetAsync($"api/robot/{robot.Id}")).StatusCode);
            var oldLogin = await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robot.Id, Secret = secret });
            Assert.Equal(HttpStatusCode.Unauthorized, oldLogin.StatusCode);
            Assert.False(string.IsNullOrEmpty(await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, newSecret)));
        }

        [Fact]
        public async Task Revoke_RejectsTokenAndLogin()
        {
            var (robot, secret) = await ProvisionRobotAsync();
            var token = await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret);

            var admin = await AdminClientAsync();
            Assert.Equal(HttpStatusCode.OK, (await admin.PostAsync($"api/robot/{robot.Id}/revoke", null)).StatusCode);

            var robotClient = AuthTestHelper.CreateBearerClient(_factory, token);
            Assert.Equal(HttpStatusCode.Unauthorized, (await robotClient.GetAsync($"api/robot/{robot.Id}")).StatusCode);
            var login = await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robot.Id, Secret = secret });
            Assert.Equal(HttpStatusCode.Unauthorized, login.StatusCode);
        }

        [Fact]
        public async Task RotateAndRevoke_AsRobot_AreForbiddenOrUnauthorized()
        {
            var (robot, secret) = await ProvisionRobotAsync();
            var robotClient = AuthTestHelper.CreateBearerClient(_factory,
                await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret));

            // AdminOnly is cookie-only, so a robot JWT doesn't even authenticate against it.
            Assert.Equal(HttpStatusCode.Unauthorized, (await robotClient.PostAsync($"api/robot/{robot.Id}/revoke", null)).StatusCode);
            Assert.Equal(HttpStatusCode.Unauthorized, (await robotClient.PostAsync($"api/robot/{robot.Id}/credentials", null)).StatusCode);
        }

        [Fact]
        public async Task PendingSecret_OldStaysValid_UntilNewIsFirstUsed()
        {
            var (robot, oldSecret) = await ProvisionRobotAsync();
            string newSecret;
            using (var scope = _factory.Services.CreateScope())
                newSecret = (await scope.ServiceProvider.GetRequiredService<IRobotService>().BeginSecretRotationAsync(robot.Id))!;
            var oldToken = await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, oldSecret);

            // Both work while the new one hasn't been used: a push that never arrives can't lock the robot out.
            Assert.False(string.IsNullOrEmpty(await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, oldSecret)));
            var newToken = await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, newSecret);

            // First use of the new secret promoted it: the old secret and its tokens are retired.
            var oldLogin = await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robot.Id, Secret = oldSecret });
            Assert.Equal(HttpStatusCode.Unauthorized, oldLogin.StatusCode);
            Assert.Equal(HttpStatusCode.Unauthorized, (await AuthTestHelper.CreateBearerClient(_factory, oldToken).GetAsync($"api/robot/{robot.Id}")).StatusCode);
            Assert.Equal(HttpStatusCode.OK, (await AuthTestHelper.CreateBearerClient(_factory, newToken).GetAsync($"api/robot/{robot.Id}")).StatusCode);
            Assert.False(string.IsNullOrEmpty(await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, newSecret)));
        }

        [Fact]
        public async Task BeginSecretRotation_RequiresAnExistingSecret()
        {
            using var scope = _factory.Services.CreateScope();
            var robots = scope.ServiceProvider.GetRequiredService<IRobotService>();
            var unprovisioned = TestDataSeeder.CreateRobot(scope.ServiceProvider.GetRequiredService<BloomDbContext>());

            Assert.Null(await robots.BeginSecretRotationAsync(unprovisioned.Id));
            Assert.Null(await robots.BeginSecretRotationAsync(Guid.NewGuid()));
        }

        [Fact]
        public async Task Revoke_AlsoDiscardsPendingSecret()
        {
            var (robot, _) = await ProvisionRobotAsync();
            string pending;
            using (var scope = _factory.Services.CreateScope())
                pending = (await scope.ServiceProvider.GetRequiredService<IRobotService>().BeginSecretRotationAsync(robot.Id))!;

            await (await AdminClientAsync()).PostAsync($"api/robot/{robot.Id}/revoke", null);

            var login = await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robot.Id, Secret = pending });
            Assert.Equal(HttpStatusCode.Unauthorized, login.StatusCode);
        }

        [Fact]
        public async Task Delete_RequiresAdmin()
        {
            var (robot, secret) = await ProvisionRobotAsync();
            var robotClient = AuthTestHelper.CreateBearerClient(_factory, await AuthTestHelper.LoginRobotAsync(_factory, robot.Id, secret));
            using var scope = _factory.Services.CreateScope();
            var slp = await TestDataSeeder.CreateAccountAsync(scope.ServiceProvider.GetRequiredService<UserManager<Account>>(), role: "SLP");
            var slpClient = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);

            Assert.Equal(HttpStatusCode.Unauthorized, (await _factory.CreateClient().DeleteAsync($"api/robot/{robot.Id}")).StatusCode);
            Assert.Equal(HttpStatusCode.Unauthorized, (await robotClient.DeleteAsync($"api/robot/{robot.Id}")).StatusCode);
            Assert.Equal(HttpStatusCode.Forbidden, (await slpClient.DeleteAsync($"api/robot/{robot.Id}")).StatusCode);

            Assert.Equal(HttpStatusCode.OK, (await (await AdminClientAsync()).DeleteAsync($"api/robot/{robot.Id}")).StatusCode);
            var login = await _factory.CreateClient().PostAsJsonAsync("api/robot/login", new { RobotId = robot.Id, Secret = secret });
            Assert.Equal(HttpStatusCode.Unauthorized, login.StatusCode);
        }

        [Fact]
        public async Task ForgedAdminJwt_IsRejected()
        {
            // Signed with the real key -- what an attacker could do if Jwt:Key ever leaked.
            var forged = AuthTestHelper.MintJwt(_factory, new[]
            {
                new Claim(ClaimTypes.NameIdentifier, Guid.NewGuid().ToString()),
                new Claim(ClaimTypes.Role, "Admin"),
            });
            var client = AuthTestHelper.CreateBearerClient(_factory, forged);

            var response = await client.GetAsync("api/robot");

            Assert.Equal(HttpStatusCode.Unauthorized, response.StatusCode);
        }
    }
}
