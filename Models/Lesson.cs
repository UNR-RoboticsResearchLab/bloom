// bloom
// Lesson.cs
// Lesson model representing a lesson in the system.
// Created: 10/21/2025

using System.ComponentModel.DataAnnotations;

namespace Bloom.Models
{
    public enum LessonType
    {
        Language,
        Speech
    }

    public class Lesson
    {
        public int Id { get; set; }
        public required string Title { get; set; }
        public string? Description { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        public LessonType LessonType { get; set; }

        // Navigation properties
        public ICollection<Assignment>? Assignments { get; set; }
    }
}
