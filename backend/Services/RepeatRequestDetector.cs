// bloom
// RepeatRequestDetector.cs
// Stateless phrase matcher for "please repeat that" style student speech,
// used to auto-queue a replay step-control command regardless of which
// lesson step is active when the student says it.

using System.Text.RegularExpressions;

namespace bloom.Services
{
    public class RepeatRequestDetector : IRepeatRequestDetector
    {
        // Multi-word phrases: matched anywhere in the utterance, so surrounding
        // words ("can you...", "...please", "um, ...") don't block a match.
        // Run through Normalize() up front so contractions ("didn't") are
        // stored the same way the input text will be — apostrophe-stripped —
        // instead of never matching.
        private static readonly string[] Phrases = new[]
        {
            "repeat that",
            "repeat it",
            "repeat what",
            "repeat please",
            "please repeat",
            "say that again",
            "say it again",
            "say again",
            "one more time",
            "what did you say",
            "what did you just say",
            "didn't hear",
            "did not hear",
            "didn't catch",
            "did not catch",
            "come again",
            "what was that",
            "read that again",
            "read it again",
            "do that again",
            "do it again",
            "play that again",
            "start over",
            "repeat",
            "could you repeat",
            "could you say that again",
            "could you say it again",
            "could you repeat that",
            "could you repeat it",
            "could you repeat what you just said",
        }.Select(Normalize).ToArray();

        // Short standalone utterances: only trusted as a repeat request when
        // they make up the entire (short) thing the student said, since these
        // words are common enough to appear legitimately in a real answer.
        private static readonly HashSet<string> StandaloneWords = new(StringComparer.OrdinalIgnoreCase)
        {
            "repeat", "again", "huh", "what", "pardon", "sorry",
        };

        private const int StandaloneWordLimit = 3;

        public bool IsRepeatRequest(string? text)
        {
            if (string.IsNullOrWhiteSpace(text))
                return false;

            var normalized = Normalize(text);
            if (normalized.Length == 0)
                return false;

            foreach (var phrase in Phrases)
            {
                if (normalized.Contains(phrase, StringComparison.Ordinal))
                    return true;
            }

            var words = normalized.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (words.Length > 0 && words.Length <= StandaloneWordLimit && words.All(StandaloneWords.Contains))
                return true;

            return false;
        }

        // Lowercase, strip punctuation, collapse whitespace — keeps phrase
        // matching resilient to "Can you repeat that?", "REPEAT!!", etc.
        private static string Normalize(string text)
        {
            var lowered = text.ToLowerInvariant();
            var stripped = Regex.Replace(lowered, @"[^a-z0-9\s]", " ");
            return Regex.Replace(stripped, @"\s+", " ").Trim();
        }
    }
}
