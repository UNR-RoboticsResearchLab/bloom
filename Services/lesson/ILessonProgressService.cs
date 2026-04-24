


using bloom.Models;
using Microsoft.EntityFrameworkCore.Storage.ValueConversion.Internal;

namespace bloom.Services
{
    public interface ILessonProgressService
    {

        Task<LessonProgress> GetByIDAsync(string id);
        Task<IEnumerable<LessonProgress>> GetByUserIdAsync(string id);
        Task<IEnumerable<LessonProgress>> GetByEmailAsync(string email);
        Task<IEnumerable<LessonProgress>> GetByUserIdWithLessonAsync(string userId);

        Task<bool> CreateAsync(LessonProgress lessonProgress);
        Task<bool> ModifyAsync(LessonProgress lessonProgress);
        Task DeleteByIdAsync(string id);
    }
}