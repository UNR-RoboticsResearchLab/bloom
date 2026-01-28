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
        public required Guid LessonId { get; set; }
        [Required]
        public required string AssignedById { get; set; }
        public DateTime AssignedDate { get; set; }
        public DateTime? DueDate { get; set; }
        public bool IsCompleted { get; set; }

        // Navigation properties
        public Account? AssignedBy { get; set; }
        public Lesson? Lesson { get; set; }
        public Account? Student { get; set; }
    }
}