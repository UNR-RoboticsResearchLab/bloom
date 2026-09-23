namespace bloom.Models
{
    /// <summary>
    /// A single execution of a lesson within a RobotSession. Each call to start a lesson
    /// mints a new LessonRun, even when the same lesson is repeated in the same session,
    /// so that interactions and SLP feedback never overlap across runs.
    /// </summary>
    public class LessonRun
    {
        public Guid Id { get; set; } = Guid.NewGuid();

        public Guid RobotSessionId { get; set; }
        public RobotSession? RobotSession { get; set; }

        public Guid LessonId { get; set; }
        public Lesson? Lesson { get; set; }

        public string? SlpId { get; set; }
        public string? StudentId { get; set; }

        // Snapshot of the student's display name for this run, resolved once at lesson
        // start (from Account.FullName for a real assigned student, or a freely-typed
        // name for the anonymous Demo flow). Used to personalize step scripts — lives
        // here rather than on Lesson/LessonStep because it's per-run instance data, not
        // reusable authored content.
        public string? StudentName { get; set; }

        public DateTime StartedAt { get; set; } = DateTime.UtcNow;
        public DateTime? EndedAt { get; set; }

        public string Status { get; set; } = "active";

        public ICollection<LessonInteraction>? Interactions { get; set; }
    }
}
