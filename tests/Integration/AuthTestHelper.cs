using System;
using System.Net.Http;
using System.Net.Http.Json;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Tests.TestHelpers;

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
    }
}
