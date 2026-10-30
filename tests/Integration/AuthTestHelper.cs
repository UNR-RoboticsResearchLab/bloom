using System;
using System.Collections.Generic;
using System.IdentityModel.Tokens.Jwt;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Net.Http.Json;
using System.Security.Claims;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Tests.TestHelpers;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.IdentityModel.Tokens;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Logs an HttpClient in as a seeded Account via the real POST /api/User/login endpoint, so
    /// it carries a real Identity application cookie on subsequent requests.
    ///
    /// Cookie auth is what most of the controllers under test actually rely on:
    /// LessonRuntimeController/RobotSessionController have no [Authorize] attribute at all, so
    /// ASP.NET Core's authentication middleware only ever authenticates the DEFAULT scheme
    /// (Identity's cookie -- see the comment in Program.cs) automatically on every request. A
    /// JWT Bearer token is only evaluated on endpoints that explicitly opt into it via
    /// [Authorize(...)] (e.g. LessonSessionController's "JwtOrCookie" policy) -- a JWT-only
    /// helper would silently leave GetCurrentUserId() null on every other controller, which is
    /// exactly the failure mode this real-login approach avoids.
    ///
    /// WebApplicationFactory.CreateClient() has cookie handling enabled by default, so the
    /// Set-Cookie header from this login call is carried automatically by the returned client.
    /// </summary>
    internal static class AuthTestHelper
    {
        public static async Task<HttpClient> CreateAuthorizedClientAsync(BloomWebApplicationFactory factory, Account account)
        {
            var client = factory.CreateClient();

            var response = await client.PostAsJsonAsync("api/User/login",
                new { Email = account.Email, Password = TestDataSeeder.DefaultPassword });

            if (!response.IsSuccessStatusCode)
            {
                var body = await response.Content.ReadAsStringAsync();
                throw new InvalidOperationException(
                    $"Test login failed for '{account.Email}': {(int)response.StatusCode} {body}");
            }

            return client;
        }

        /// <summary>
        /// Logs a robot in via the real POST /api/robot/login with its RobotId + Secret, and
        /// returns the raw JWT. Use for robot-side endpoints and the shell relay.
        /// </summary>
        public static async Task<string> LoginRobotAsync(BloomWebApplicationFactory factory, Guid robotId, string secret)
        {
            var client = factory.CreateClient();
            var response = await client.PostAsJsonAsync("api/robot/login", new { RobotId = robotId, Secret = secret });
            if (!response.IsSuccessStatusCode)
            {
                var body = await response.Content.ReadAsStringAsync();
                throw new InvalidOperationException(
                    $"Test robot login failed for '{robotId}': {(int)response.StatusCode} {body}");
            }
            using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
            return json.RootElement.GetProperty("token").GetString()!;
        }

        public static HttpClient CreateBearerClient(BloomWebApplicationFactory factory, string token)
        {
            var client = factory.CreateClient();
            client.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", token);
            return client;
        }

        /// <summary>
        /// Signs an arbitrary JWT with the factory's real Jwt:Key -- for tests that need a token
        /// the backend would never issue itself (forged roles, stale token versions, expiry).
        /// </summary>
        public static string MintJwt(BloomWebApplicationFactory factory, IEnumerable<Claim> claims, DateTime? expires = null)
        {
            var key = factory.Services.GetRequiredService<IConfiguration>()["Jwt:Key"]!;
            var token = new JwtSecurityToken(
                claims: claims,
                expires: expires ?? DateTime.UtcNow.AddMinutes(30),
                signingCredentials: new SigningCredentials(
                    new SymmetricSecurityKey(Encoding.UTF8.GetBytes(key)), SecurityAlgorithms.HmacSha256));
            return new JwtSecurityTokenHandler().WriteToken(token);
        }
    }
}
