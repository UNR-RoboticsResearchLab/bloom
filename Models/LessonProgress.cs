// bloom
// LessonProgress.cs
// Tracks a student's progress on a specific lesson.

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class LessonProgress
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        [Required]
        public required Guid LessonId { get; set; }
        public Lesson? Lesson { get; set; }

        [Required]
        public required string StudentId { get; set; }
        public Account? Student { get; set; }

        public int LessonStep { get; set; }
        public int ProgressPercentage { get; set; }
        public DateTime LastUpdated { get; set; } = DateTime.UtcNow;
    }
}
