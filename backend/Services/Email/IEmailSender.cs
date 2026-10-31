// bloom
// IEmailSender.cs
// Sends one HTML email. Implemented over SMTP; tests substitute a fake.

namespace bloom.Services.Email
{
    public interface IEmailSender
    {
        Task SendEmailAsync(string toEmail, string subject, string htmlBody);
    }
}
