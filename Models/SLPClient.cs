// bloom
// SLPClient.cs
// Associates a student with one or more SLP teacher accounts.

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

        public DateTime CreatedDate { get; set; } = DateTime.UtcNow;

        public ICollection<Account>? Teachers { get; set; }
        public ICollection<Assignment>? Assignments { get; set; }
    }
}
