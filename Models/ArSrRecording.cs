namespace bloom.Models
{
    /// <summary>
    /// Audio recording for one sentence within an RSR session.
    /// </summary>
    public class ArSrRecording
    {
        public Guid Id { get; set; } = Guid.NewGuid();

        public Guid SessionId { get; set; }
        public ArSrSession? Session { get; set; }

        /// <summary>Sentence number 1–16 this recording corresponds to.</summary>
        public int SentenceNumber { get; set; }

        public string FilePath { get; set; } = string.Empty;
    }
}
