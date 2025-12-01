// bloom
// ClassroomService.cs
// Assingment service business logic layer
// Created: 11/21/25

using bloom.Data;
using bloom.Models;
using System.Collections.Generic;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class ClassroomService : IClassroomService
    {
        private readonly BloomDbContext _dbContext;
        private readonly IAccountService _accountService;

        public ClassroomService(BloomDbContext context, IAccountService accountService)
        {
            _dbContext = context;
            _accountService = accountService;
        }

        public async Task<bool> CreateAsync(string name, string teacherId)
        {
            if(name.Length < 3)
            {
                throw new Exception("name not long enough");
            }

            var teacher = await _accountService.GetByIdAsync(teacherId);
            if (teacher == null)
            {
                throw new Exception("teacher not found");
            }

            var classroom = new Classroom
            {
                Name = name
            };
            classroom.Teachers.Add(teacher);

            await _dbContext.Classrooms.AddAsync(classroom);
            return true;
        }

        public async Task DeleteByIdAsync(Guid id)
        {
            var classroom = await _dbContext.Classrooms.FindAsync(id);
            if (classroom != null)
            {
                _dbContext.Classrooms.Remove(classroom);
                await _dbContext.SaveChangesAsync();
            }
        }

        public async Task<Classroom?> GetByIdAsync(Guid id)
        {
            return await _dbContext.Classrooms
                .Include(c => c.Students)
                .Include(c => c.Teachers)
                .Include(c => c.Assignments)
                .Include(c => c.Lessons)
                .FirstOrDefaultAsync(c => c.Id == id);
        }

        public async Task<bool> ModifyAsync(Classroom classroom)
        {
            var existingClassroom = await _dbContext.Classrooms.FindAsync(classroom.Id);
            if (existingClassroom == null)
            {
                return false;
            }

            existingClassroom.Name = classroom.Name;
            existingClassroom.AccentColor = classroom.AccentColor;
            existingClassroom.BackgroundImageUrl = classroom.BackgroundImageUrl;
            existingClassroom.UpdatedDate = DateTime.Now;

            _dbContext.Classrooms.Update(existingClassroom);
            await _dbContext.SaveChangesAsync();
            return true;
        }
    }
}