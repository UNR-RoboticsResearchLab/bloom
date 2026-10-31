using System;
using System.Net;
using System.Net.Http;
using System.Net.Http.Json;
using System.Text.Json;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>Admin user and role management (api/admin/users).</summary>
    public class AdminUserManagementTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public AdminUserManagementTests(BloomWebApplicationFactory factory) => _factory = factory;

        private async Task<(HttpClient Client, Account Admin)> AdminClientAsync()
        {
            using var scope = _factory.Services.CreateScope();
            var um = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var admin = await TestDataSeeder.CreateAdminAsync(um);
            return (await AuthTestHelper.CreateAuthorizedClientAsync(_factory, admin), admin);
        }

        private async Task<Account> SeedAsync(string role)
        {
            using var scope = _factory.Services.CreateScope();
            var um = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var acct = await TestDataSeeder.CreateAccountAsync(um, role: role);
            await um.AddToRoleAsync(acct, role);
            return acct;
        }

        private async Task<bool> InRoleAsync(string id, string role)
        {
            using var scope = _factory.Services.CreateScope();
            var um = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            return await um.IsInRoleAsync((await um.FindByIdAsync(id))!, role);
        }

        [Fact]
        public async Task List_AsNonAdmin_ReturnsForbidden()
        {
            var slp = await SeedAsync("SLP");
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);

            Assert.Equal(HttpStatusCode.Forbidden, (await client.GetAsync("api/admin/users")).StatusCode);
        }

        [Fact]
        public async Task List_Anonymous_IsRejected()
        {
            var response = await _factory.CreateClient().GetAsync("api/admin/users");
            Assert.True(response.StatusCode is HttpStatusCode.Unauthorized or HttpStatusCode.Redirect or HttpStatusCode.Forbidden);
        }

        [Fact]
        public async Task List_AsAdmin_IncludesUsersWithIdentityRole()
        {
            var (client, _) = await AdminClientAsync();
            var slp = await SeedAsync("SLP");

            var response = await client.GetAsync("api/admin/users");

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
            var row = Assert.Single(json.RootElement.EnumerateArray(), e => e.GetProperty("id").GetString() == slp.Id);
            Assert.Equal("SLP", row.GetProperty("role").GetString());
        }

        [Fact]
        public async Task SetRole_ChangesIdentityRoleAndColumn()
        {
            var (client, _) = await AdminClientAsync();
            var user = await SeedAsync("SLP");

            var response = await client.PutAsJsonAsync($"api/admin/users/{user.Id}/role", new { Role = "teacher" });

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            Assert.True(await InRoleAsync(user.Id, "Teacher"));
            Assert.False(await InRoleAsync(user.Id, "SLP"));
        }

        [Fact]
        public async Task SetRole_Promote_ToAdmin_GrantsAdminAccess()
        {
            var (client, _) = await AdminClientAsync();
            var user = await SeedAsync("SLP");

            await client.PutAsJsonAsync($"api/admin/users/{user.Id}/role", new { Role = "Admin" });

            var promoted = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, user);
            Assert.Equal(HttpStatusCode.OK, (await promoted.GetAsync("api/admin/users")).StatusCode);
        }

        [Fact]
        public async Task SetRole_UnknownRole_ReturnsBadRequest()
        {
            var (client, _) = await AdminClientAsync();
            var user = await SeedAsync("SLP");

            var response = await client.PutAsJsonAsync($"api/admin/users/{user.Id}/role", new { Role = "Robot" });

            Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
        }

        [Fact]
        public async Task SetRole_OwnAccount_ReturnsConflict()
        {
            var (client, admin) = await AdminClientAsync();

            var response = await client.PutAsJsonAsync($"api/admin/users/{admin.Id}/role", new { Role = "SLP" });

            Assert.Equal(HttpStatusCode.Conflict, response.StatusCode);
            Assert.True(await InRoleAsync(admin.Id, "Admin"));
        }

        [Fact]
        public async Task SetRole_MissingUser_ReturnsNotFound()
        {
            var (client, _) = await AdminClientAsync();

            var response = await client.PutAsJsonAsync($"api/admin/users/{Guid.NewGuid()}/role", new { Role = "SLP" });

            Assert.Equal(HttpStatusCode.NotFound, response.StatusCode);
        }

        [Fact]
        public async Task Delete_RemovesUser()
        {
            var (client, _) = await AdminClientAsync();
            var user = await SeedAsync("Student");

            var response = await client.DeleteAsync($"api/admin/users/{user.Id}");

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            using var scope = _factory.Services.CreateScope();
            var um = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            Assert.Null(await um.FindByIdAsync(user.Id));
        }

        [Fact]
        public async Task Delete_OwnAccount_ReturnsConflict()
        {
            var (client, admin) = await AdminClientAsync();

            Assert.Equal(HttpStatusCode.Conflict, (await client.DeleteAsync($"api/admin/users/{admin.Id}")).StatusCode);
        }

        [Fact]
        public async Task Delete_AsNonAdmin_ReturnsForbidden()
        {
            var slp = await SeedAsync("SLP");
            var victim = await SeedAsync("Student");
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);

            Assert.Equal(HttpStatusCode.Forbidden, (await client.DeleteAsync($"api/admin/users/{victim.Id}")).StatusCode);
        }
    }
}
