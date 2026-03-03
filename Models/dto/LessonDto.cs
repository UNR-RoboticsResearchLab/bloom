namespace bloom.Models.dto
{
    public class LessonDto
    {
        public required string Title { get; set; }
        public string? Description { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        public LessonType LessonType { get; set; }

        public string? LessonDescription { get; set; }
        public required string CreatedById { get; set; }
    }
}