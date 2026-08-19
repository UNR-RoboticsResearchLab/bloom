// bloom
// Lesson.cs
// Lesson model representing a lesson in the system.

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public enum LessonType
    {
        Language,
        Speech
    }

    public class Lesson
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();
        public required string Title { get; set; }
        public string? Description { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        public int TotalSteps { get; set; }

        // Was previously a bare field (not a property), which EF Core silently never
        // mapped -- objectives were accepted on create/update requests and echoed back
        // in that same response, but never actually persisted. Stored as JSON via the
        // value converter configured in BloomDbContext.
        public List<string?> LearningObjectives { get; set; } = [];

        [Required]
        public required string CreatedById { get; set; }
        public Account? CreatedBy { get; set; }

        public LessonType LessonType { get; set; }

        // Navigation properties
        public ICollection<LessonStep> Steps { get; set; } = new List<LessonStep>();
        public ICollection<Assignment>? Assignments { get; set; }
    }
}