// bloom
// IClassroomService.cs
// Interface defining behavior of LessonService.cs (WIP)
// Created: 10/22/2025

using bloom.Models;

namespace bloom.Services
{
    public interface IClassroomService
    {
        // Get
        Task<Classroom?> GetByIdAsync(Guid id);

        // Create
        Task<bool> CreateAsync(string name, string teacherId);

        // Modify
        Task<bool> ModifyAsync(Classroom classroom);

        // Delete
        Task DeleteByIdAsync(Guid id);
    }
}