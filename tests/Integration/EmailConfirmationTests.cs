using System;
using System.Linq;
using System.Net;
using System.Net.Http.Json;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Identity;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Registration sends a confirmation link, the link confirms the account, and sign-in is refused
    /// until then. Mail goes to FakeEmailSender, never to SMTP.
    /// </summary>
    public class EmailConfirmationTests : IClassFixture<BloomWebApplicationFactory>
    {
        private readonly BloomWebApplicationFactory _factory;

        public EmailConfirmationTests(BloomWebApplicationFactory factory)
        {
            _factory = factory;
        }

        private async Task<string> RegisterAsync(System.Net.Http.HttpClient client)
        {
            var email = $"confirm-{Guid.NewGuid():N}@test.bloom";
            var response = await client.PostAsJsonAsync("api/User/create", new
            {
                FullName = "Confirm Me",
                Email = email,
                UserName = email,
                Password = TestDataSeeder.DefaultPassword,
                SelectedRole = "SLP"
            });
            response.EnsureSuccessStatusCode();
            return email;
        }

        private FakeEmailSender.SentEmail LastMailTo(string email) =>
            _factory.Services.GetRequiredService<FakeEmailSender>().Sent.Last(m => m.To == email);

        private static string LinkIn(FakeEmailSender.SentEmail mail)
        {
            var match = Regex.Match(mail.HtmlBody, "href=\"([^\"]+)\"");
            Assert.True(match.Success, "confirmation email has no link");
            return System.Net.WebUtility.HtmlDecode(match.Groups[1].Value);
        }

        private Task<System.Net.Http.HttpResponseMessage> LoginAsync(System.Net.Http.HttpClient client, string email) =>
            client.PostAsJsonAsync("api/User/login", new { Email = email, Password = TestDataSeeder.DefaultPassword });

        [Fact]
        public async Task Register_SendsConfirmationEmail_AndLinkConfirmsAccount()
        {
            var client = _factory.CreateClient();
            var email = await RegisterAsync(client);

            var mail = LastMailTo(email);
            Assert.Equal("Confirm your Bloom account", mail.Subject);

            // Unconfirmed accounts cannot sign in.
            var blocked = await LoginAsync(client, email);
            Assert.Equal(HttpStatusCode.BadRequest, blocked.StatusCode);
            Assert.Contains("EmailNotConfirmed", await blocked.Content.ReadAsStringAsync());

            var confirm = await client.GetAsync(new Uri(LinkIn(mail)).PathAndQuery);
            Assert.Equal(HttpStatusCode.OK, confirm.StatusCode);
            Assert.Contains("Email confirmed", await confirm.Content.ReadAsStringAsync());

            using var scope = _factory.Services.CreateScope();
            var account = await scope.ServiceProvider.GetRequiredService<UserManager<Account>>().FindByEmailAsync(email);
            Assert.True(account!.EmailConfirmed);

            Assert.Equal(HttpStatusCode.OK, (await LoginAsync(client, email)).StatusCode);
        }

        [Fact]
        public async Task ConfirmEmail_TamperedToken_Fails()
        {
            var client = _factory.CreateClient();
            var email = await RegisterAsync(client);
            var uri = new Uri(LinkIn(LastMailTo(email)));

            var bad = await client.GetAsync(uri.PathAndQuery + "AAAA");

            Assert.Contains("Confirmation failed", await bad.Content.ReadAsStringAsync());
            Assert.Equal(HttpStatusCode.BadRequest, (await LoginAsync(client, email)).StatusCode);
        }

        [Fact]
        public async Task Resend_UnconfirmedAccount_SendsNewEmail_UnknownAddress_SendsNothing()
        {
            var client = _factory.CreateClient();
            var email = await RegisterAsync(client);
            var sender = _factory.Services.GetRequiredService<FakeEmailSender>();
            var before = sender.Sent.Count(m => m.To == email);

            var resend = await client.PostAsJsonAsync("api/User/resend-confirmation", new { Email = email });
            var unknown = await client.PostAsJsonAsync("api/User/resend-confirmation", new { Email = "nobody@test.bloom" });

            Assert.Equal(HttpStatusCode.OK, resend.StatusCode);
            Assert.Equal(HttpStatusCode.OK, unknown.StatusCode);
            Assert.Equal(before + 1, sender.Sent.Count(m => m.To == email));
            Assert.DoesNotContain(sender.Sent, m => m.To == "nobody@test.bloom");
        }

        [Fact]
        public async Task Resend_IsRateLimited()
        {
            var client = _factory.CreateClient();
            HttpStatusCode last = HttpStatusCode.OK;
            for (var i = 0; i < 6; i++)
                last = (await client.PostAsJsonAsync("api/User/resend-confirmation", new { Email = "x@test.bloom" })).StatusCode;

            Assert.Equal(HttpStatusCode.TooManyRequests, last);
        }

        [Fact]
        public async Task ForgotPassword_SendsLink_AndFormSetsNewPassword()
        {
            using (var scope = _factory.Services.CreateScope())
            {
                var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
                var account = await TestDataSeeder.CreateAccountAsync(userManager, role: "SLP");
                _resetEmail = account.Email!;
            }
            var client = _factory.CreateClient();

            var response = await client.PostAsJsonAsync("api/User/forgot-password", new { Email = _resetEmail });
            Assert.Equal(HttpStatusCode.OK, response.StatusCode);

            var mail = LastMailTo(_resetEmail);
            Assert.Equal("Reset your Bloom password", mail.Subject);
            var uri = new Uri(LinkIn(mail));
            var query = System.Web.HttpUtility.ParseQueryString(uri.Query);

            var form = await client.GetAsync(uri.PathAndQuery);
            Assert.Contains("Choose a new password", await form.Content.ReadAsStringAsync());

            const string newPassword = "NewPassw0rd!";
            var post = await client.PostAsync("api/User/reset-password", new System.Net.Http.FormUrlEncodedContent(new[]
            {
                new System.Collections.Generic.KeyValuePair<string, string>("userId", query["userId"]!),
                new System.Collections.Generic.KeyValuePair<string, string>("token", query["token"]!),
                new System.Collections.Generic.KeyValuePair<string, string>("password", newPassword),
            }));
            Assert.Contains("Password changed", await post.Content.ReadAsStringAsync());

            var login = await client.PostAsJsonAsync("api/User/login", new { Email = _resetEmail, Password = newPassword });
            Assert.Equal(HttpStatusCode.OK, login.StatusCode);
        }

        [Fact]
        public async Task ForgotPassword_UnknownAddress_SendsNothing()
        {
            var client = _factory.CreateClient();

            var response = await client.PostAsJsonAsync("api/User/forgot-password", new { Email = "ghost@test.bloom" });

            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            Assert.DoesNotContain(_factory.Services.GetRequiredService<FakeEmailSender>().Sent, m => m.To == "ghost@test.bloom");
        }

        [Fact]
        public async Task ResetPassword_BadToken_Fails()
        {
            var client = _factory.CreateClient();

            var post = await client.PostAsync("api/User/reset-password", new System.Net.Http.FormUrlEncodedContent(new[]
            {
                new System.Collections.Generic.KeyValuePair<string, string>("userId", "nope"),
                new System.Collections.Generic.KeyValuePair<string, string>("token", "AAAA"),
                new System.Collections.Generic.KeyValuePair<string, string>("password", "NewPassw0rd!"),
            }));

            Assert.Contains("invalid or has expired", await post.Content.ReadAsStringAsync());
        }

        [Fact]
        public async Task CreateStudent_RequiresConfirmation_AndSendsEmail()
        {
            using var scope = _factory.Services.CreateScope();
            var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
            var slp = await TestDataSeeder.CreateAccountAsync(userManager, role: "SLP");
            var client = await AuthTestHelper.CreateAuthorizedClientAsync(_factory, slp);
            var email = $"student-{Guid.NewGuid():N}@test.bloom";

            var response = await client.PostAsJsonAsync("api/User/student", new { FullName = "Kid", Email = email });
            response.EnsureSuccessStatusCode();

            Assert.Equal("Confirm your Bloom account", LastMailTo(email).Subject);
            var temporaryPassword = System.Text.Json.JsonDocument.Parse(await response.Content.ReadAsStringAsync())
                .RootElement.GetProperty("temporaryPassword").GetString();
            Assert.Contains(temporaryPassword!, LastMailTo(email).HtmlBody);
            var student = await userManager.FindByEmailAsync(email);
            Assert.False(student!.EmailConfirmed);
        }

        private string _resetEmail = "";
    }
}
