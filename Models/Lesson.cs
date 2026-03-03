// bloom
// Lesson.cs
// Lesson model representing a lesson in the system.
// Created: 10/21/2025

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    
    // should be able to add more lesson types in the future as needed,
    // but for now we'll just have these two categories to keep it simple
    public enum LessonType
    {
        Language,
        SpeechTherapy
    }

    public class Lesson
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();
        public required string Title { get; set; }
        public string? Description { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }


        public required string LessonFileUrl { get; set; }

        [Required]
        public required string CreatedById { get; set; }
        public required Account CreatedBy { get; set; }

        public LessonType LessonType { get; set; }

        // Navigation properties
        public ICollection<Assignment>? Assignments { get; set; }
    }
}
