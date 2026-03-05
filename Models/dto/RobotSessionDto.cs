// bloom
// RobotSessionDto.cs
// Data transfer objects for RobotSession API operations
// Created: 11/18/2025

using System.Text.Json.Serialization;

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for starting a new robot session
    /// </summary>
    public class StartSessionDto
    {
        /// <summary>
        /// Whether to create an anonymous session (null UserId)
        /// </summary>
        [JsonPropertyName("anonymous")]
        public bool Anonymous { get; set; } = false;
        /// <summary>
        /// RobotId to associate with the session
        /// </summary>
        [JsonPropertyName("robot_id")]
        public Guid RobotId { get; set; }
        /// <summary>
        /// Optional UserId for the session
        /// </summary>
        [JsonPropertyName("user_id")]
        public string? UserId { get; set; }
    }

    /// <summary>
    /// Response DTO for session information
    /// </summary>
    public class RobotSessionResponseDto
    {
        public Guid Id { get; set; }
        public string? UserId { get; set; }
        public DateTime CreatedAt { get; set; }
        public DateTime LastUpdatedAt { get; set; }
        public int Robots { get; set; }
        public List<Guid> RobotIds { get; set; } = new();
        public RobotState? LastState { get; set; }
    }

    /// <summary>
    /// Request DTO for robot state input (used for both adding and updating states)
    /// </summary>
    public class RobotStateDto
    {
        public string? Status { get; set; } = "";
        public string? CurrentTask { get; set; } = "";
        public int? CurrentBehaviorId { get; set; }
        public string? SpeechLog { get; set; } = "";
    }

    /// <summary>
    /// Response DTO for robot state history
    /// </summary>
    public class RobotStateHistoryDto
    {
        public Guid Id { get; set; }
        public Guid RobotSessionId { get; set; }
        public Guid RobotId { get; set; }
        public string? Status { get; set; }
        public string? CurrentTask { get; set; }
        public DateTime Timestamp { get; set; }
    }
}
