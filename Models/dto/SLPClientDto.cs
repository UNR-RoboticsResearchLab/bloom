// bloom
// SLPClientDto.cs
// DTOs for SLPClient (student-SLP association) API operations

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for creating a new SLPClient profile, associating a student with the requesting SLP.
    /// </summary>
    public class CreateSLPClientDto
    {
        public required string Name { get; set; }
        public required string StudentId { get; set; }
    }

    /// <summary>
    /// Response DTO for an SLPClient record.
    /// </summary>
    public class SLPClientResponseDto
    {
        public Guid Id { get; set; }
        public string Name { get; set; } = string.Empty;
        public string StudentId { get; set; } = string.Empty;
        public string? StudentName { get; set; }
        public List<AssignmentResponseDto> Assignments { get; set; } = [];
        public List<StudentLessonProgressDto> Progress { get; set; } = [];
    }
}
