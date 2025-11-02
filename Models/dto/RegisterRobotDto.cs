using System;

namespace bloom.Models
{
    public class RegisterRobotDto
    {
        public required string Name { get; set; }
        public required string Model { get; set; }
        public required string SerialNumber { get; set; }
        public required DateTime ManufactureDate { get; set; }
        public required string FirmwareVersion { get; set; }
        public required string IPAddress { get; set; }
        public string? RegisteredUserId { get; set; }

        // RSA keys ??        
    }
}