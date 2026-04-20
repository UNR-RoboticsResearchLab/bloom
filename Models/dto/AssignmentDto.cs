// bloom
// AssignmentDto.cs
// DTOs for lesson assignment API operations

namespace bloom.Models.dto
{
    /// <summary>
    /// Request DTO for creating a lesson assignment for a student.
    /// </summary>
    public class CreateAssignmentDto
    {
        public required string StudentId { get; set; }
        public required Guid LessonId { get; set; }
        public DateTime? DueDate { get; set; }
    }

    /// <summary>
    /// Response DTO for an assignment record.
    /// </summary>
    public class AssignmentResponseDto
    {
        public Guid Id { get; set; }
        public string StudentId { get; set; } = string.Empty;
        public Guid LessonId { get; set; }
        public string LessonTitle { get; set; } = string.Empty;
        public string AssignedById { get; set; } = string.Empty;
        public DateTime AssignedDate { get; set; }
        public DateTime? DueDate { get; set; }
        public bool IsCompleted { get; set; }
    }
}
