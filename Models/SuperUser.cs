// bloom
// SuperUser.cs
// SuperUser model representing a SuperUser for system administration

using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public class SuperUser : IdentityUser
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