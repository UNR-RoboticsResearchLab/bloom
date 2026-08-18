// bloom
// RobotProfile.cs
// Model representing a student's personal customization of their robot
// (nickname, personality, etc.), decoupled from the physical Robot device.
// Created: 08/17/2026

using System.ComponentModel.DataAnnotations;

namespace bloom.Models
{
    public class RobotProfile
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();

        public required string AccountId { get; set; }
        public Account? Account { get; set; }

        /// <summary>
        /// The name the student has given their robot. Purely cosmetic — distinct
        /// from Robot.Name, which doubles as the physical device's login identifier.
        /// </summary>
        [MaxLength(50)]
        public required string Nickname { get; set; }

        /// <summary>
        /// One of the presets in <see cref="RobotPersonality.All"/>.
        /// </summary>
        public required string PersonalityTrait { get; set; }

        [MaxLength(200)]
        public string? Catchphrase { get; set; }

        /// <summary>
        /// Hex color or preset theme name used to accent the robot's UI for this student.
        /// </summary>
        public string? ColorTheme { get; set; }

        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }
    }
}
