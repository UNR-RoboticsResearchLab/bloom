// bloom
// ILessonService.cs
// Interface defining behavior of LessonService.cs (WIP)
// Created: 10/22/2025

using bloom.Models;

namespace bloom.Services
{
    public interface ILessonService
    {
        // Get
        Task<Lesson> GetByIdAsync(int id);
        Task<IEnumerable<Lesson>> GetByUserIdAsync(string id);
        Task<IEnumerable<Lesson>> GetByEmailAsync(string email);

        // Create
        Task<bool> CreateAsync(Lesson lesson);


        // Modify
        Task<bool> ModifyAsync(Lesson lesson);

        // Delete
        Task DeleteByIdAsync(int id);
    }
}