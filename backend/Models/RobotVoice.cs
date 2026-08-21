// bloom
// RobotVoice.cs
// Fixed list of TTS voice presets selectable for a RobotProfile or as a
// live per-session override.

namespace bloom.Models
{
    public static class RobotVoice
    {
        public record VoicePreset(string Id, string Label);

        public static readonly VoicePreset[] All =
        {
            new("en-US-AvaNeural", "Ava — Warm, friendly"),
            new("en-US-JennyNeural", "Jenny — Bright, upbeat"),
            new("en-US-GuyNeural", "Guy — Calm, steady"),
            new("en-US-AriaNeural", "Aria — Expressive"),
            new("en-US-DavisNeural", "Davis — Friendly, deep"),
            new("en-US-JaneNeural", "Jane — Gentle, soft"),
        };

        public const string Default = "en-US-AvaNeural";

        public static bool IsValid(string voiceId) => All.Any(v => v.Id == voiceId);
    }
}
