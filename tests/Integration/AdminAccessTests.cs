using System;
using System.Net;
using System.Net.Http.Json;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Admin grants remote shell on every robot, so the ways of becoming one are locked down:
    /// public sign-up can create any role except Admin.
    /// </summary>
    public class AdminAccessTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public AdminAccessTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private static object NewAccount(string role)
        {
            var email = $"signup-{Guid.NewGuid():N}@test.bloom";
            return new { FullName = "Sign Up", Email = email, UserName = email, Password = TestDataSeeder.DefaultPassword, SelectedRole = role };
        }

        [Fact]
        public async Task Create_AdminRole_Anonymous_ReturnsForbidden()
        {
            var client = _factory.CreateClient();

            var response = await client.PostAsJsonAsync("api/User/create", NewAccount("Admin"));

            Assert.Equal(HttpStatusCode.Forbidden, response.StatusCode);
        }

        [Fact]
        public async Task Create_AdminRole_AsNonAdmin_ReturnsForbidden()
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var slp = await TestDataSeeder.CreateAccountAsync(userManager, role: "SLP");
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);

            var response = await client.PostAsJsonAsync("api/User/create", NewAccount("admin"));

            Assert.Equal(HttpStatusCode.Forbidden, response.StatusCode);
        }

        [Fact]
        public async Task Create_AdminRole_AsAdmin_Succeeds()
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var admin = await TestDataSeeder.CreateAdminAsync(userManager);
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, admin);

            var response = await client.PostAsJsonAsync("api/User/create", NewAccount("Admin"));

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        }

        [Fact]
        public async Task Create_NonAdminRole_Anonymous_StillSucceeds()
        {
            var client = _factory.CreateClient();

            var response = await client.PostAsJsonAsync("api/User/create", NewAccount("Student"));

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        }
    }
}
