// bloom
// FacilitatorUser.cs
// FacilitatorUser model representing a Parent or an unaffiliated user.
// Created: 10/21/2025


using Microsoft.AspNetCore.Identity;

namespace Bloom.Models
{
    public class FacilitatorUser : Account
    {  
        public ICollection<Assignment>? Assignments { get; set; }
        public ICollection<Lesson>? Lessons { get; set; }

        // Navigation properties
        public ICollection<StudentUser>? Students { get; set; }
    }
}