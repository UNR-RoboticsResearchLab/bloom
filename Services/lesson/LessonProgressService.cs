

using bloom.Data;
using bloom.Models;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{

    class LessonProgressService : ILessonProgressService
    {

        private readonly BloomDbContext _context;
        private readonly IAccountService _accountService;

        public LessonProgressService(BloomDbContext context, IAccountService accountService)
        {
            _context = context;
            _accountService = accountService;
        }



        public Task<bool> CreateAsync(LessonProgress progress)
        {
            LessonProgress newLesson = new LessonProgress
            {
                LessonId = progress.LessonId,
                StudentId = progress.StudentId,
                ProgressPercentage = progress.ProgressPercentage,
                LessonStep = progress.LessonStep,
                LastUpdated = DateTime.UtcNow
            };

            _context.LessonProgresses.Add(newLesson);
            _context.SaveChanges();

            return Task.FromResult(true);
        }

        public Task DeleteByIdAsync(string id)
        {
            var progress = _context.LessonProgresses.FirstOrDefault(p => p.Id.ToString() == id);
            if (progress != null)
            {
                _context.LessonProgresses.Remove(progress);
                _context.SaveChanges();
            }
            return Task.CompletedTask;
        }

        public async Task<IEnumerable<LessonProgress>> GetByUserIdAsync(string userId)
        {
            var lessonProgresses = await _context.LessonProgresses.Include(p => p.Student).Where(p => p.StudentId == userId).ToListAsync();
            return lessonProgresses;
        }

        public async Task<IEnumerable<LessonProgress>> GetByEmailAsync(string email)
        {

            var lessonProgresses = await _context.LessonProgresses
                .Include(p => p.Student)
                .Where(p => p.Student != null && p.Student.Email == email)
                .ToListAsync();


            return lessonProgresses;
        }

        public async Task<IEnumerable<LessonProgress>> GetByUserIdWithLessonAsync(string userId)
        {
            return await _context.LessonProgresses
                .Include(p => p.Lesson)
                .Where(p => p.StudentId == userId)
                .ToListAsync();
        }

        public async Task<LessonProgress> GetByIDAsync(string id)
        {
            var guid = Guid.Parse(id);
            var lessonProgress = await _context.LessonProgresses.FirstOrDefaultAsync(p => p.Id == guid);

            if (lessonProgress == null)
            {
                throw new Exception("Lesson progress not found.");
            }

            return lessonProgress;
        }

        public async Task<bool> ModifyAsync(LessonProgress progress)
        {
            ArgumentNullException.ThrowIfNull(progress);

            if (progress.Id == Guid.Empty)
            {
                throw new ArgumentException("Lesson progress Id cannot be empty.", nameof(progress));
            }

            LessonProgress? existingProgress = await _context.LessonProgresses.FirstOrDefaultAsync(p => p.Id == progress.Id);

            if (existingProgress == null)
            {
                throw new Exception("Lesson progress not found.");
            }

            existingProgress.ProgressPercentage = progress.ProgressPercentage;
            existingProgress.LessonStep = progress.LessonStep;
            existingProgress.LastUpdated = DateTime.UtcNow;

            _context.SaveChanges();
            return true;
        }
    }
}