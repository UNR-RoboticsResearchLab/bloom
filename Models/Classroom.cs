// bloom
// Class.cs
// Class model representing a class of StudentUsers and 'elevated users' in the system.
// Created: 10/21/2025


using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class Classroom
    {
        [Required]
        public required string Id { get; set; }
        [Required]
        public required string Name { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }
        public string? AccentColor { get; set; }
        public string? BackgroundImageUrl {get; set; }

        // Navigation properties
        public ICollection<Account>? Students { get; set; }
        public required ICollection<Account> Teachers { get; set; }
    }
}