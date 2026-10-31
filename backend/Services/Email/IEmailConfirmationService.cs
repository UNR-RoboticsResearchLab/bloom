// bloom
// IEmailConfirmationService.cs
// Issues, emails and redeems email confirmation tokens.

using bloom.Models;

namespace bloom.Services.Email
{
    public interface IEmailConfirmationService
    {
        /// <summary>
        /// Emails a confirmation link to the account's address. When a temporary password is given
        /// (accounts created for someone else) it is included so they can sign in after confirming.
        /// </summary>
        Task SendConfirmationAsync(Account account, string? temporaryPassword = null);

        /// <summary>
        /// Emails a new link if the address belongs to an unconfirmed account.
        /// Returns silently otherwise so callers cannot probe which addresses are registered.
        /// </summary>
        Task ResendConfirmationAsync(string email);

        /// <summary>Confirms the account from a link's userId and Base64Url token.</summary>
        Task<bool> ConfirmEmailAsync(string userId, string encodedToken);
    }
}
