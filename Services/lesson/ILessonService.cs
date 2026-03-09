// bloom
// ILessonService.cs
// Interface defining behavior of LessonService.cs (WIP)
// Created: 10/22/2025

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

        // Create
        Task<bool> CreateAsync(LessonDto lesson);


        // Modify
        Task<bool> ModifyAsync(Lesson lesson);

        // Delete
        Task DeleteByIdAsync(string id);
    }
}