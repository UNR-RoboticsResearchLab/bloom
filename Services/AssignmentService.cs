// bloom
// AssignmentService.cs
// Assingment service business logic layer
// Created: 11/21/25

using bloom.Data;
using bloom.Models;
using System.Collections.Generic;
using Microsoft.EntityFrameworkCore;
using Microsoft.AspNetCore.Razor.TagHelpers;

namespace bloom.Services
{
    public class AssignmentService : IAssignmentService
    {
        private readonly BloomDbContext _dbContext;
        private readonly IAccountService _accountService;

        public AssignmentService(BloomDbContext context, IAccountService accountService)
        {
            _dbContext = context;
            _accountService = accountService;
        }

        public async Task<bool> AssignToStudent(Guid lessonId, string studentId, string assignedById)
        {
            if (await _accountService.GetByIdAsync(studentId) == null)
            {
                return false;
            }
            if (await _accountService.GetByIdAsync(assignedById) == null)
            {
                return false;
            }

            var assignment = new Assignment
            {
                StudentId = studentId,
                LessonId = lessonId,
                AssignedById = assignedById
            };

            await _dbContext.Assignments.AddAsync(assignment);
            await _dbContext.SaveChangesAsync();
            return true;
        }

        public async Task<bool> Create(Assignment assignment)
        {
            await _dbContext.Assignments.AddAsync(assignment);
            await _dbContext.SaveChangesAsync();
            return true;
        }

        public async Task DeleteById(Guid id)
        {
            var assignment = await _dbContext.Assignments.Where(a => a.Id == id).FirstOrDefaultAsync();

            if (assignment != null)
            {
                _dbContext.Assignments.Remove(assignment);
                await _dbContext.SaveChangesAsync();
            }
        }

        public async Task<Assignment?> GetByIdAsync(Guid id)
        {
            var assignment = await _dbContext.Assignments.Where(a => a.Id == id).FirstOrDefaultAsync();
            return assignment;
        }

        public async Task<bool> Modify(Assignment assignment)
        {
            var to_modify = await _dbContext.Assignments.Where(a => a.Id == assignment.Id).FirstOrDefaultAsync();

            to_modify = assignment;

            await _dbContext.SaveChangesAsync();
            return true;
        }
    }
}