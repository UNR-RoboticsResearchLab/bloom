// bloom
// StudentUser.cs
// StudentUser model representing a student user.
// Created: 10/21/2025

using System.ComponentModel.DataAnnotations;
using Microsoft.AspNetCore.Identity;

namespace bloom.Models
{
    public class StudentUser : Account
    {
        public required string AccessId { get; set; }

        
        [Required]
        public string CreatedById { get; set; }
        public required Account CreatedBy { get; set; }

        // Navigation properties
        public ICollection<Assignment>? Assignments { get; set; }
    }
}