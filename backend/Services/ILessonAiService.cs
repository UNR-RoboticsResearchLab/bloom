// bloom
// ILessonAiService.cs
// Interface defining behavior of LessonAiService.cs

using bloom.Models.dto;

namespace bloom.Services
{
    public interface ILessonAiService
    {
        // Generates a full lesson (title, objectives, steps) from a free-text prompt.
        // Returns a LessonDto for the caller to preview/edit in the builder — never persists directly.
        Task<LessonDto> GenerateLessonAsync(LessonAiGenerateRequestDto request);

        // Generates or revises a single step within the context of the rest of the lesson.
        Task<LessonStepDto> GenerateStepAsync(LessonAiGenerateStepRequestDto request);
    }
}
