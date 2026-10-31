// bloom
// PasswordResetService.cs
// Builds the password reset link, emails it, and redeems it.

using System.Net;
using System.Text;
using bloom.Models;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.WebUtilities;

namespace bloom.Services.Email
{
    public class PasswordResetService : IPasswordResetService
    {
        private readonly UserManager<Account> _userManager;
        private readonly IEmailSender _emailSender;
        private readonly IEmailConfirmationService _emailConfirmation;
        private readonly IHttpContextAccessor _httpContextAccessor;
        private readonly IConfiguration _configuration;

        public PasswordResetService(
            UserManager<Account> userManager,
            IEmailSender emailSender,
            IEmailConfirmationService emailConfirmation,
            IHttpContextAccessor httpContextAccessor,
            IConfiguration configuration)
        {
            _userManager = userManager;
            _emailSender = emailSender;
            _emailConfirmation = emailConfirmation;
            _httpContextAccessor = httpContextAccessor;
            _configuration = configuration;
        }

        public async Task SendResetLinkAsync(string email)
        {
            var account = await _userManager.FindByEmailAsync(email);
            if (account == null || string.IsNullOrEmpty(account.Email))
                return;

            // An unconfirmed address is not proven to be theirs yet; confirm it first.
            if (!await _userManager.IsEmailConfirmedAsync(account))
            {
                await _emailConfirmation.SendConfirmationAsync(account);
                return;
            }

            var token = await _userManager.GeneratePasswordResetTokenAsync(account);
            var encodedToken = WebEncoders.Base64UrlEncode(Encoding.UTF8.GetBytes(token));
            var link = $"{PublicUrl.GetBaseUrl(_configuration, _httpContextAccessor.HttpContext)}/api/user/reset-password" +
                       $"?userId={Uri.EscapeDataString(account.Id)}&token={encodedToken}";

            var body = $"<p>Hello {DisplayName(account)},</p>" +
                       "<p>We received a request to reset your Bloom password. Use this link to choose a new one:</p>" +
                       $"<p><a href=\"{WebUtility.HtmlEncode(link)}\">Reset my password</a></p>" +
                       "<p>This link expires in 24 hours. If you did not ask for this, you can ignore this message; your password has not changed.</p>";

            await _emailSender.SendEmailAsync(account.Email, "Reset your Bloom password", body);
        }

        public async Task<IdentityResult> ResetPasswordAsync(string userId, string encodedToken, string newPassword)
        {
            var invalid = IdentityResult.Failed(new IdentityError
            {
                Description = "This reset link is invalid or has expired. Request a new one."
            });

            if (string.IsNullOrEmpty(userId) || string.IsNullOrEmpty(encodedToken))
                return invalid;

            var account = await _userManager.FindByIdAsync(userId);
            if (account == null)
                return invalid;

            string token;
            try
            {
                token = Encoding.UTF8.GetString(WebEncoders.Base64UrlDecode(encodedToken));
            }
            catch (FormatException)
            {
                return invalid;
            }

            var result = await _userManager.ResetPasswordAsync(account, token, newPassword);
            // Password rule violations are worth showing; a bad token is not.
            if (!result.Succeeded && result.Errors.Any(e => e.Code == nameof(IdentityErrorDescriber.InvalidToken)))
                return invalid;
            return result;
        }

        private static string DisplayName(Account account) =>
            WebUtility.HtmlEncode(account.FullName ?? account.Email ?? "");
    }
}
