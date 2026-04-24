using bloom.Models.dto;

namespace bloom.Models.dto
{
    public class LessonDto
    {
        public string? Id { get; set; }
        public required string Title { get; set; }
        public string? Description { get; set; }
        public DateTime CreatedDate { get; set; }
        public DateTime? UpdatedDate { get; set; }

        public LessonType LessonType { get; set; }

        // Set server-side from auth claims — never trusted from the request body
        public string? CreatedById { get; set; }

        // JSON array string of learning objectives — populated on response, optional on create
        public string? LearningObjectives { get; set; }

        // Structured steps — populated on response; used for direct creation
        public List<LessonStepDto>? Steps { get; set; }

        // Raw JSON import — if provided on create and Steps is empty, parsed to populate Steps
        public string? LessonDescription { get; set; }
    }
}