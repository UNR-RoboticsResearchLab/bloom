// bloom
// SmtpOptions.cs
// Settings for the outgoing mail server, bound from the "Smtp" config section.

namespace bloom.Services.Email
{
    public class SmtpOptions
    {
        public const string SectionName = "Smtp";

        /// <summary>SMTP host. Empty means email is not configured and sending fails.</summary>
        public string Host { get; set; } = "";

        /// <summary>Submission port; the connection is upgraded with STARTTLS.</summary>
        public int Port { get; set; } = 587;

        /// <summary>Login name. Leave empty for a relay that does not require authentication.</summary>
        public string User { get; set; } = "";

        public string Password { get; set; } = "";

        /// <summary>Sender address, e.g. "Bloom &lt;noreply@example.edu&gt;" or "noreply@example.edu".</summary>
        public string From { get; set; } = "";
    }
}
