// bloom
// Account.cs
// Account model representing a speech-language pathologist or a teacher user.
// Created: 10/21/2025


using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public class Account : IdentityUser
    {
        public required string FullName { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }
    }
}