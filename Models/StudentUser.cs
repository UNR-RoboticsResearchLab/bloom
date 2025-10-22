// bloom
// StudentUser.cs
// StudentUser model representing a student user.
// Created: 10/21/2025

using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public class StudentUser : IdentityUser
    {
        public int Id { get; set; }
        public string Email { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        // Navigation properties
        public ICollection<Assignment> Assignments { get; set; }
    }
}