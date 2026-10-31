// bloom
// IPasswordResetService.cs
// Emails and redeems password reset links.

using Microsoft.AspNetCore.Identity;

namespace bloom.Services.Email
{
    public interface IPasswordResetService
    {
        /// <summary>
        /// Emails a reset link to a confirmed account; an unconfirmed account gets a confirmation
        /// link instead. Returns silently for unknown addresses so callers cannot probe for accounts.
        /// </summary>
        Task SendResetLinkAsync(string email);

        /// <summary>Sets a new password from a link's userId and Base64Url token.</summary>
        Task<IdentityResult> ResetPasswordAsync(string userId, string encodedToken, string newPassword);
    }
}
