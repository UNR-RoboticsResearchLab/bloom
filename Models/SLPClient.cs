// bloom
// SLPClient.cs
// Associates a student with the SLP responsible for them.

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class SLPClient
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        [Required]
        public required string Name { get; set; }

        [Required]
        public required string StudentId { get; set; }
        public Account? Student { get; set; }

        [Required]
        public required string SlpId { get; set; }
        public Account? Slp { get; set; }

        public DateTime CreatedDate { get; set; } = DateTime.UtcNow;

        public ICollection<Assignment>? Assignments { get; set; }
    }
}
