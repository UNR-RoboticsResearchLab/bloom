// bloom
// Behavior.cs
// Behavior model representing a behavior record for a blossom robot.
// Created: 10/21/2025

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class Behavior
    {
        public int Id { get; set; }
        [Required]
        public required string Name { get; set; }
        public string? Description { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        public string? BehaviorType { get; set; }
        public double? Scaling { get; set; }
    }
}