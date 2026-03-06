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
        Task<Assignment?> GetByIdAsync(Guid id);

        // Create
        Task<bool> Create(Assignment assignment);

        // Manage

        Task<bool> AssignToStudent(Guid lessonId, string studentId, string assignedById);
        Task<bool> Modify(Assignment assignment);



        // Remove
        Task DeleteById(Guid id);

    }   
    
}