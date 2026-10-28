// bloom
// Robot.cs
// Model representing a Robot entity
// Created: 10/22/2025

using System.ComponentModel.DataAnnotations;
using System.Text.Json.Serialization;

namespace bloom.Models
{
    public class Robot
    {
        [Key]
        public Guid Id { get; set; } = Guid.NewGuid();
        
        public required string Name { get; set; }
        public string? Model { get; set; }
        public DateTime ManufactureDate { get; set; }
        public string? FirmwareVersion { get; set; }
        public string? IPAddress { get; set; }

        public string? RegisteredUserId { get; set; }

        public Account? RegisteredUser { get; set; }

        // PasswordHasher hash of the robot's login secret. Null means the robot can't log in
        // (never provisioned, or revoked). Never serialized: Robot is returned directly by the API.
        [JsonIgnore]
        public string? SecretHash { get; set; }

        // Embedded in every robot JWT as the "tv" claim. Bumping it (rotate/revoke) invalidates
        // every token issued before, on the next request.
        public int TokenVersion { get; set; }

        // Reported by the robot agent on every connect (SemVer 2.0 from Nerdbank.GitVersioning,
        // plus the git commit). Distinct from FirmwareVersion, which bloom_node self-reports.
        public string? AgentVersion { get; set; }
        public string? AgentCommit { get; set; }
        public DateTime? LastSeenAt { get; set; }

        // Navigation properties can be added here
    }
}