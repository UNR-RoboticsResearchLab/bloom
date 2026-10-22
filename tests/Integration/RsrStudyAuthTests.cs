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

namespace bloom.Tests.Integration;

public class RsrStudyAuthTests : IClassFixture<BloomWebApplicationFactory>
{
    private readonly BloomWebApplicationFactory _factory;
    public RsrStudyAuthTests(BloomWebApplicationFactory factory) { _factory = factory; }
    private async Task<Account> Account(string role, bool membership = true)
    {
        using var scope = _factory.Services.CreateScope();
        var users = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
        var user = await TestDataSeeder.CreateAccountAsync(users, role);
        if (membership) Assert.True((await users.AddToRoleAsync(user, role)).Succeeded);
        return user;
    }
    private static Task<HttpResponseMessage> Login(HttpClient client, Account user, string password = TestDataSeeder.DefaultPassword) =>
        client.PostAsJsonAsync("/api/rsr-study/auth/login", new { login = user.UserName, password });

    [Fact]
    public async Task IssuedResearcherCanLoginReachStudyAndLogout()
    {
        var user = await Account("Researcher");
        using var client = _factory.CreateClient();
        Assert.Equal(HttpStatusCode.Unauthorized, (await client.GetAsync("/api/rsr-study/auth/session")).StatusCode);
        Assert.Equal(HttpStatusCode.OK, (await Login(client, user)).StatusCode);
        var session = await client.GetAsync("/api/rsr-study/auth/session");
        Assert.Equal(HttpStatusCode.OK, session.StatusCode);
        var profile = await session.Content.ReadFromJsonAsync<JsonElement>();
        Assert.Equal("Researcher", profile.GetProperty("role").GetString());
        Assert.Equal(user.Id, profile.GetProperty("id").GetString());
        Assert.Equal(HttpStatusCode.NoContent, (await client.PostAsync("/api/rsr-study/auth/logout", null)).StatusCode);
        Assert.Equal(HttpStatusCode.Unauthorized, (await client.GetAsync("/api/rsr-study/auth/session")).StatusCode);
    }
    [Fact]
    public async Task WrongPasswordAndUnassignedRolesDoNotReceiveStudySession()
    {
        var researcher = await Account("Researcher");
        var slp = await Account("SLP");
        var forgedProfile = await Account("Researcher", membership: false);
        using var client = _factory.CreateClient();
        foreach (var (user, password) in new[] { (researcher, "Wrong@12345"), (slp, TestDataSeeder.DefaultPassword), (forgedProfile, TestDataSeeder.DefaultPassword) })
        {
            Assert.Equal(HttpStatusCode.Unauthorized, (await Login(client, user, password)).StatusCode);
            Assert.Equal(HttpStatusCode.Unauthorized, (await client.GetAsync("/api/rsr-study/auth/session")).StatusCode);
        }
    }
    [Fact]
    public async Task OnlyAnAdministratorCanIssueCredentials()
    {
        var admin = await Account("Admin");
        var researcher = await Account("Researcher");
        using var client = _factory.CreateClient();
        var assigned = new { login = "rsr-" + Guid.NewGuid().ToString("N"), password = "Assigned@456", fullName = "Study Researcher" };
        Assert.Equal(HttpStatusCode.Unauthorized, (await client.PostAsJsonAsync("/api/rsr-study/auth/researchers", assigned)).StatusCode);
        await Login(client, researcher);
        Assert.Equal(HttpStatusCode.Forbidden, (await client.PostAsJsonAsync("/api/rsr-study/auth/researchers", assigned)).StatusCode);
        await Login(client, admin);
        var created = await client.PostAsJsonAsync("/api/rsr-study/auth/researchers", assigned);
        Assert.Equal(HttpStatusCode.OK, created.StatusCode);
        Assert.Equal(HttpStatusCode.Conflict, (await client.PostAsJsonAsync("/api/rsr-study/auth/researchers", assigned)).StatusCode);
        using var issuedClient = _factory.CreateClient();
        Assert.Equal(HttpStatusCode.OK, (await issuedClient.PostAsJsonAsync("/api/rsr-study/auth/login", new { login = assigned.login, password = assigned.password })).StatusCode);
    }
    [Fact]
    public async Task PublicSignupCannotGrantAdministrativeOrResearcherAccess()
    {
        using var client = _factory.CreateClient();
        foreach (var role in new[] { "Admin", "Researcher" })
        {
            var response = await client.PostAsJsonAsync("/api/user/create", new {
                username = "blocked-" + Guid.NewGuid().ToString("N"), email = Guid.NewGuid()+"@test.bloom",
                password = "Attempt@123", fullName = "Unassigned", selectedRole = role
            });
            Assert.False(response.IsSuccessStatusCode);
        }
    }
}
