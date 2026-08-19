// bloom
// ILessonService.cs
// Interface defining behavior of LessonService.cs

using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public interface ILessonService
    {
        // Get
        Task<Lesson?> GetByIdAsync(string id);
        Task<IEnumerable<Lesson>> GetByUserIdAsync(string id);
        Task<IEnumerable<Lesson>> GetByEmailAsync(string email);

        Task<IEnumerable<Lesson>> GetAllAsync();

        // Create -- returns the persisted entity (with its generated Id) so callers can
        // act on it immediately, e.g. running the just-created lesson on a robot.
        Task<Lesson?> CreateAsync(LessonDto lesson);

        // Modify
        Task<bool> ModifyAsync(Lesson lesson);

        // Full update -- title/description/type/objectives and the complete step list
        // (replaces existing steps rather than diffing, matching how the lesson builder
        // always submits the full current draft).
        Task<bool> UpdateAsync(LessonDto lesson);

        // Delete
        Task DeleteByIdAsync(string id);

        // Steps
        Task<bool> RemoveStepAsync(Guid lessonId, Guid stepId);
    }
}