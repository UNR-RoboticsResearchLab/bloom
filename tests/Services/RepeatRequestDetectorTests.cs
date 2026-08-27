using Xunit;
using bloom.Services;

namespace bloom.Tests.Services
{
    /// <summary>
    /// Unit tests for RepeatRequestDetector: recognizing "please repeat that"
    /// style student speech regardless of phrasing/punctuation/case, while not
    /// misfiring on ordinary lesson answers.
    /// </summary>
    public class RepeatRequestDetectorTests
    {
        private readonly RepeatRequestDetector _detector = new();

        [Theory]
        [InlineData("Can you repeat that?")]
        [InlineData("could you repeat that please")]
        [InlineData("Say that again")]
        [InlineData("say it again!")]
        [InlineData("one more time")]
        [InlineData("What did you say?")]
        [InlineData("i didn't hear you")]
        [InlineData("I did not catch that")]
        [InlineData("come again?")]
        [InlineData("What was that")]
        [InlineData("REPEAT!!")]
        [InlineData("again")]
        [InlineData("huh?")]
        [InlineData("pardon")]
        [InlineData("  again  ")]
        public void IsRepeatRequest_RecognizesRepeatPhrasing(string text)
        {
            Assert.True(_detector.IsRepeatRequest(text));
        }

        [Theory]
        [InlineData(null)]
        [InlineData("")]
        [InlineData("   ")]
        [InlineData("A dog says woof")]
        [InlineData("The answer is four")]
        [InlineData("yes")]
        [InlineData("no")]
        [InlineData("I like the color blue")]
        // Long enough that "again"/"repeat" appearing as ordinary content
        // shouldn't trip the short-standalone-word path.
        [InlineData("I want to try that game again with my friend")]
        public void IsRepeatRequest_DoesNotMisfireOnOrdinaryResponses(string? text)
        {
            Assert.False(_detector.IsRepeatRequest(text));
        }
    }
}
