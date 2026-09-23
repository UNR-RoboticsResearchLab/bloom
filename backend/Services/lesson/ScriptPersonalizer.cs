// bloom
// ScriptPersonalizer.cs
// Substitutes the {name} placeholder token in lesson step/interaction scripts with a
// student's name, so the same authored content personalizes both the robot TTS path
// and the frontend display path from a single implementation.

namespace bloom.Services
{
    public static class ScriptPersonalizer
    {
        private const string NameToken = "{name}";
        private const string FallbackName = "friend";

        public static string? Apply(string? script, string? studentName)
        {
            if (string.IsNullOrEmpty(script))
                return script;

            var name = string.IsNullOrWhiteSpace(studentName) ? FallbackName : studentName.Trim();
            return script.Replace(NameToken, name, StringComparison.OrdinalIgnoreCase);
        }
    }
}
