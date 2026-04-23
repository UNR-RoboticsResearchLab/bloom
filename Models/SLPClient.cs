// bloom
// Class.cs
// Class model representing a class of StudentUsers and 'elevated users' in the system.
// Created: 10/21/2025


using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class SLPClient
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();
        [Required]
        public required string Name { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }
        public string? AccentColor { get; set; }
        public string? BackgroundImageUrl { get; set; }

        

        // Navigation properties
        public required string StudentId { get; set; }
        public Account? Student { get; set; }
        public ICollection<Account>? Teachers { get; set; }
        public ICollection<Assignment>? Assignments { get; set; }
        public ICollection<Lesson>? Lessons { get; set; }
    }
}