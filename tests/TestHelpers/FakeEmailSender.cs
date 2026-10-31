using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading.Tasks;
using bloom.Services.Email;

namespace bloom.Tests.TestHelpers
{
    /// <summary>Records outgoing email instead of sending it, so tests never touch an SMTP server.</summary>
    public class FakeEmailSender : IEmailSender
    {
        public record SentEmail(string To, string Subject, string HtmlBody);

        private readonly ConcurrentQueue<SentEmail> _sent = new();

        public IReadOnlyCollection<SentEmail> Sent => _sent;

        public Task SendEmailAsync(string toEmail, string subject, string htmlBody)
        {
            _sent.Enqueue(new SentEmail(toEmail, subject, htmlBody));
            return Task.CompletedTask;
        }
    }
}
