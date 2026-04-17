using bloom.Models;
using Microsoft.EntityFrameworkCore.Storage.ValueConversion.Internal;

namespace bloom.Services
{
    public interface ILessonInteractionService
    {
        
        Task<LessonInteraction> GetByIDAsync(string id);
        Task<IEnumerable<LessonInteraction>> GetByUserIdAsync(string id);
        Task<IEnumerable<LessonInteraction>> GetByEmailAsync(string email);
        
        Task<bool> CreateAsync(LessonInteraction interaction);
        Task<bool> ModifyAsync(LessonInteraction interaction);
        Task DeleteByIdAsync(string id);
    }
}