// bloom
// Assignment.cs
// Assignment model representing a lesson assigned to a student.
// Created: 10/21/2025

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class Assignment
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();
        [Required]
        public required string StudentId { get; set; }
        [Required]
        public required string LessonId { get; set; }
        [Required]
        public required string AssignedById { get; set; }
        public DateTime AssignedDate { get; set; }
        public DateTime? DueDate { get; set; }
        public bool IsCompleted { get; set; }

        // Navigation properties
        public required Account AssignedBy { get; set; }
        public required Lesson Lesson { get; set; }
        public required Account Student { get; set; }
    }
}