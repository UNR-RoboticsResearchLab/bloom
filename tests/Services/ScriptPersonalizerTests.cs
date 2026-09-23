using Xunit;
using bloom.Services;

namespace bloom.Tests.Services
{
    /// <summary>
    /// Unit tests for ScriptPersonalizer: substituting the {name} placeholder token in
    /// lesson scripts with a resolved student name, or a neutral fallback when none is
    /// available.
    /// </summary>
    public class ScriptPersonalizerTests
    {
        [Fact]
        public void Apply_ReplacesToken_WithStudentName()
        {
            var result = ScriptPersonalizer.Apply("Hi {name}, let's practice!", "Milo");
            Assert.Equal("Hi Milo, let's practice!", result);
        }

        [Theory]
        [InlineData("{NAME}")]
        [InlineData("{Name}")]
        [InlineData("{name}")]
        public void Apply_MatchesToken_CaseInsensitively(string token)
        {
            var result = ScriptPersonalizer.Apply($"Great job, {token}!", "Milo");
            Assert.Equal("Great job, Milo!", result);
        }

        [Fact]
        public void Apply_ReplacesAllOccurrences()
        {
            var result = ScriptPersonalizer.Apply("{name}, are you ready {name}?", "Milo");
            Assert.Equal("Milo, are you ready Milo?", result);
        }

        [Theory]
        [InlineData(null)]
        [InlineData("")]
        [InlineData("   ")]
        public void Apply_FallsBackToNeutralTerm_WhenNoNameAvailable(string? studentName)
        {
            var result = ScriptPersonalizer.Apply("Hi {name}!", studentName);
            Assert.Equal("Hi friend!", result);
        }

        [Fact]
        public void Apply_TrimsWhitespaceAroundName()
        {
            var result = ScriptPersonalizer.Apply("Hi {name}!", "  Milo  ");
            Assert.Equal("Hi Milo!", result);
        }

        [Fact]
        public void Apply_LeavesScriptUnchanged_WhenNoTokenPresent()
        {
            var result = ScriptPersonalizer.Apply("No token here.", "Milo");
            Assert.Equal("No token here.", result);
        }

        [Theory]
        [InlineData(null)]
        [InlineData("")]
        public void Apply_PassesThroughNullOrEmptyScript(string? script)
        {
            var result = ScriptPersonalizer.Apply(script, "Milo");
            Assert.Equal(script, result);
        }
    }
}
