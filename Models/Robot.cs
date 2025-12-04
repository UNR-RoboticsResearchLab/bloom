// bloom
// Robot.cs
// Model representing a Robot entity
// Created: 10/22/2025

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class Robot
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();
        
        public required string Name { get; set; }
        public required string Model { get; set; }
        public required string SerialNumber { get; set; }
        public required DateTime ManufactureDate { get; set; }
        public required string FirmwareVersion { get; set; }
        public required string IPAddress { get; set; }

        public string? RegisteredUserId { get; set; }

        public Account? RegisteredUser { get; set; }

        // Navigation properties can be added here
    }
}