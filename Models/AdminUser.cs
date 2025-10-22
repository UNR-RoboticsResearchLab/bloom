// bloom
// AdminUser.cs
// AdminUser model representing a speech-language pathologist or a teacher user.
// Created: 10/21/2025


using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public class AdminUser : IdentityUser
    {
        public int Id { get; set; }
        public string FirstName { get; set; }
        public string LastName { get; set; }
        public string Email { get; set; }
        public string PhoneNumber { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        // Navigation properties
        public ICollection<StudentUser> Students { get; set; }
    }
}