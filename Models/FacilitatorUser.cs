// bloom
// FacilitatorUser.cs
// FacilitatorUser model representing a Parent or an unaffiliated user.
// Created: 10/21/2025


using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public class FacilitatorUser : IdentityUser
    {
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