// bloom
// ISLPClientService.cs
// Interface defining behavior of SLPClientService
// Created: 02/21/2026

using bloom.Models;

namespace bloom.Services
{
    public interface ISLPClientService
    {
        // Get
        Task<SLPClient?> GetByIdAsync(Guid id);
        Task<IEnumerable<SLPClient>> GetAllAsync();
        Task<IEnumerable<SLPClient>> GetByTeacherIdAsync(string teacherId);
        Task<IEnumerable<SLPClient>> GetByStudentIdAsync(string studentId);

        // Create
        Task<bool> CreateAsync(string name, string teacherId, string studentId);

        // Modify
        Task<bool> ModifyAsync(SLPClient slpClient);

        // Delete
        Task DeleteByIdAsync(Guid id);

        // Teacher management
        Task<bool> AddTeacherAsync(Guid slpClientId, string teacherId);
        Task<bool> RemoveTeacherAsync(Guid slpClientId, string teacherId);

        // Lesson management
        Task<bool> AddLessonAsync(Guid slpClientId, Guid lessonId);
        Task<bool> RemoveLessonAsync(Guid slpClientId, Guid lessonId);

        // Assignment management
        Task<bool> AddAssignmentAsync(Guid slpClientId, Guid assignmentId);
        Task<bool> RemoveAssignmentAsync(Guid slpClientId, Guid assignmentId);
    }
}
