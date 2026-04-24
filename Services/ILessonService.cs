// bloom
// ILessonService.cs
// Interface defining behavior of LessonService.cs

using bloom.Models;

namespace bloom.Services
{
    public interface ILessonService
    {
        // Get
        Task<Lesson?> GetByIdAsync(string id);
        Task<IEnumerable<Lesson>> GetByUserIdAsync(string id);
        Task<IEnumerable<Lesson>> GetByEmailAsync(string email);

        // Create
        Task<bool> CreateAsync(LessonDto lesson);

        // Modify
        Task<bool> ModifyAsync(Lesson lesson);

        // Delete
        Task DeleteByIdAsync(string id);

        // Steps
        Task<bool> RemoveStepAsync(Guid lessonId, Guid stepId);
    }
}