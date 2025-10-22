// bloom
// Assignment.cs
// Assignment model representing a lesson assigned to a student.
// Created: 10/21/2025

using System.ComponentModel.DataAnnotations;

namespace Bloom.Models
{
    public class Assignment
    {
        public int Id { get; set; }
        [Required]
        public string StudentId { get; set; }
        [Required]
        public int LessonId { get; set; }
        [Required]
        public string AssignedById { get; set; }
        public DateTime AssignedDate { get; set; }
        public DateTime? DueDate { get; set; }
        public bool IsCompleted { get; set; }

        // Navigation properties
        public required StudentUser Student { get; set; }
        public required Lesson Lesson { get; set; }
        public required Account AssignedBy { get; set; }
    }
}