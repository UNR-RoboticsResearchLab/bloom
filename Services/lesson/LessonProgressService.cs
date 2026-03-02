

using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{

    class LessonProgressService : ILessonProgressService
    {

        private readonly BloomDbContext _context;

        public LessonProgressService(BloomDbContext context)
        {
            _context = context;
        }



        public Task<bool> CreateAsync(LessonProgress progress)
        {
            LessonProgress newLesson = new LessonProgress
            {
                LessonId = progress.LessonId,
                StudentId = progress.StudentId,
                ProgressPercentage = progress.ProgressPercentage,
                LessonStep = progress.LessonStep,
                TotalSteps = progress.TotalSteps,
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
            Guid guid = Guid.Parse(userId);
            var lessonProgresses = await _context.LessonProgresses.Include(p => p.Student).Where(p => p.StudentId == guid).ToListAsync();
            return lessonProgresses;
        }

        public async Task<IEnumerable<LessonProgress>> GetByEmailAsync(string email)
        {
            var lessonProgresses = await _context.LessonProgresses
                .Include(p => p.Student)
                .Where(p => p.Student.Email == email)
                .ToListAsync();

            
            return lessonProgresses;
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
            LessonProgress existingProgress = await _context.LessonProgresses.FirstOrDefaultAsync(p => p.Id == progress.Id);
            
            if (existingProgress == null)
            {
                throw new Exception("Lesson progress not found.");
            }

            existingProgress.ProgressPercentage = progress.ProgressPercentage;
            existingProgress.LessonStep = progress.LessonStep;
            existingProgress.TotalSteps = progress.TotalSteps;
            existingProgress.LastUpdated = DateTime.UtcNow;

            _context.SaveChanges();
            return true;
        }
    }
}