// bloom
// SmtpEmailSender.cs
// IEmailSender over SMTP using MailKit (STARTTLS).

using MailKit.Net.Smtp;
using MailKit.Security;
using Microsoft.Extensions.Options;
using MimeKit;

namespace bloom.Services.Email
{
    public class SmtpEmailSender : IEmailSender
    {
        private readonly SmtpOptions _options;

        public SmtpEmailSender(IOptions<SmtpOptions> options)
        {
            _options = options.Value;
        }

        public async Task SendEmailAsync(string toEmail, string subject, string htmlBody)
        {
            if (string.IsNullOrWhiteSpace(_options.Host) || string.IsNullOrWhiteSpace(_options.From))
                throw new InvalidOperationException(
                    "Email is not configured. Set Smtp:Host and Smtp:From (env Smtp__Host, Smtp__From).");

            var message = new MimeMessage();
            message.From.Add(MailboxAddress.Parse(_options.From));
            message.To.Add(MailboxAddress.Parse(toEmail));
            message.Subject = subject;
            message.Body = new BodyBuilder { HtmlBody = htmlBody }.ToMessageBody();

            using var client = new SmtpClient();
            await client.ConnectAsync(_options.Host, _options.Port, SecureSocketOptions.StartTls);
            if (!string.IsNullOrEmpty(_options.User))
                await client.AuthenticateAsync(_options.User, _options.Password);
            await client.SendAsync(message);
            await client.DisconnectAsync(true);
        }
    }
}
