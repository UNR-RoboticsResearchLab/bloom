// bloom
// EmailConfirmationService.cs
// Builds the confirmation link, emails it, and redeems it.

using System.Net;
using System.Text;
using bloom.Models;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.WebUtilities;

namespace bloom.Services.Email
{
    public class EmailConfirmationService : IEmailConfirmationService
    {
        private readonly UserManager<Account> _userManager;
        private readonly IEmailSender _emailSender;
        private readonly IHttpContextAccessor _httpContextAccessor;
        private readonly IConfiguration _configuration;

        public EmailConfirmationService(
            UserManager<Account> userManager,
            IEmailSender emailSender,
            IHttpContextAccessor httpContextAccessor,
            IConfiguration configuration)
        {
            _userManager = userManager;
            _emailSender = emailSender;
            _httpContextAccessor = httpContextAccessor;
            _configuration = configuration;
        }

        public async Task SendConfirmationAsync(Account account, string? temporaryPassword = null)
        {
            if (string.IsNullOrEmpty(account.Email))
                throw new InvalidOperationException("Account has no email address.");

            var token = await _userManager.GenerateEmailConfirmationTokenAsync(account);
            var encodedToken = WebEncoders.Base64UrlEncode(Encoding.UTF8.GetBytes(token));
            var link = $"{PublicUrl.GetBaseUrl(_configuration, _httpContextAccessor.HttpContext)}/api/user/confirm-email" +
                       $"?userId={Uri.EscapeDataString(account.Id)}&token={encodedToken}";

            var passwordHtml = temporaryPassword == null ? "" :
                $"<p>An account was created for you. Your temporary password is: <strong>{WebUtility.HtmlEncode(temporaryPassword)}</strong><br>" +
                "Sign in with this email address and password once you have confirmed, then change the password.</p>";

            var body = $"<p>Hello {WebUtility.HtmlEncode(account.FullName ?? account.Email)},</p>" +
                       "<p>Welcome to Bloom. Please confirm your email address to finish setting up your account:</p>" +
                       $"<p><a href=\"{WebUtility.HtmlEncode(link)}\">Confirm my email</a></p>" +
                       passwordHtml +
                       "<p>This link expires in 24 hours. If you did not create a Bloom account, you can ignore this message.</p>";

            await _emailSender.SendEmailAsync(account.Email, "Confirm your Bloom account", body);
        }

        public async Task ResendConfirmationAsync(string email)
        {
            var account = await _userManager.FindByEmailAsync(email);
            if (account == null || await _userManager.IsEmailConfirmedAsync(account))
                return;

            await SendConfirmationAsync(account);
        }

        public async Task<bool> ConfirmEmailAsync(string userId, string encodedToken)
        {
            if (string.IsNullOrEmpty(userId) || string.IsNullOrEmpty(encodedToken))
                return false;

            var account = await _userManager.FindByIdAsync(userId);
            if (account == null)
                return false;

            string token;
            try
            {
                token = Encoding.UTF8.GetString(WebEncoders.Base64UrlDecode(encodedToken));
            }
            catch (FormatException)
            {
                return false;
            }

            var result = await _userManager.ConfirmEmailAsync(account, token);
            return result.Succeeded;
        }
    }
}
