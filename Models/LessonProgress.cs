

namespace bloom.Models
{
    public class LessonProgress
    {
        public Guid Id { get; set; } = Guid.NewGuid();
        
        
        public Guid LessonId { get; set; }
        public Lesson? Lesson { get; set; }
        public Guid StudentId { get; set; }
        public Account? Student { get; set; }

        
        public int ProgressPercentage { get; set; }
        public int LessonStep { get; set; }
        public int TotalSteps { get; set; }

        public DateTime LastUpdated { get; set; } = DateTime.UtcNow;

    }
}