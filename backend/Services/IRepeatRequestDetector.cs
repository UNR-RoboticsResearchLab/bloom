namespace bloom.Services
{
    /// <summary>
    /// Recognizes whether a piece of transcribed student speech is a request
    /// for the robot to repeat itself (e.g. "can you repeat that?", "again!"),
    /// independent of whatever step/interaction it was said during.
    /// </summary>
    public interface IRepeatRequestDetector
    {
        bool IsRepeatRequest(string? text);
    }
}
