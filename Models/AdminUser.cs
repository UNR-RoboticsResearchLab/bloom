// bloom
// AdminUser.cs
// AdminUser model representing a speech-language pathologist or a teacher user.
// Created: 10/21/2025


using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public enum AccountType
    {
        teacher,
        slp
    }
    public class AdminUser : Account
    {
        public ICollection<Assignment>? Assignments { get; set; }
        // Navigation properties
        public ICollection<StudentUser>? Students { get; set; }
        public ICollection<Lesson>? Lessons { get; set; }

        public AccountType accountType { get; set; }
    }
}