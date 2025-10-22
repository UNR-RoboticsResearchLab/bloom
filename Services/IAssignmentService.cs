// bloom
// IAssignmentService.cs
// Interface defining behavior of AssignmentService.cs (WIP)
// Created: 10/22/2025

using bloom.Models;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.SignalR;

namespace bloom.Services
{
    public interface IAssignmentService
    {
        // Get
        Task<Assignment> GetByIdAsync(int id);
        Task<IEnumerable<Assignment>> GetByStudentAsync(StudentUser user);
        Task<IEnumerable<Assignment>> GetByClassroomIdAsync(int id);

        // Create
        Task<bool> Create(Assignment assignment);

        // Manage

        Task<bool> AssignToStudent(Assignment assignment, StudentUser user);
        Task<bool> Modify(Assignment assignment);



        // Remove
        Task DeleteById(int id);

    }   
    
}