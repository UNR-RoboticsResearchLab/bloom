

using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class LessonService : ILessonService
    {
        private readonly BloomDbContext _context;

        public LessonService(BloomDbContext dbContext)
        {
            _context = dbContext;
        }

        public async Task<bool> CreateAsync(Lesson lesson)
        {
            try
            {
                ArgumentNullException.ThrowIfNull(lesson);

                lesson.CreatedDate = DateTime.UtcNow;
                _context.Lessons.Add(lesson);
                await _context.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error creating lesson: {ex.Message}");
                return false;
            }
        }

        public async Task DeleteByIdAsync(string id)
        {
            try
            {
                if (string.IsNullOrEmpty(id))
                {
                    throw new ArgumentNullException(nameof(id));
                }

                var lesson = await _context.Lessons.FindAsync(new Guid(id));
                if (lesson != null)
                {
                    _context.Lessons.Remove(lesson);
                    await _context.SaveChangesAsync();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error deleting lesson: {ex.Message}");
            }
        }

        public async Task<IEnumerable<Lesson>> GetByEmailAsync(string email)
        {
            try
            {
                if (string.IsNullOrEmpty(email))
                {
                    throw new ArgumentNullException(nameof(email));
                }

                var lessons = await _context.Lessons
                    .Where(l => l.CreatedBy.Email == email)
                    .ToListAsync();

                return lessons;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lessons by email: {ex.Message}");
                return Enumerable.Empty<Lesson>();
            }
        }

        public async Task<Lesson> GetByIdAsync(string id)
        {
            try
            {
                if (string.IsNullOrEmpty(id))
                {
                    throw new ArgumentNullException(nameof(id));
                }

                var lesson = await _context.Lessons
                    .Include(l => l.CreatedBy)
                    .Include(l => l.Assignments)
                    .FirstOrDefaultAsync(l => l.Id == new Guid(id));

                if (lesson == null)
                {
                    throw new KeyNotFoundException($"Lesson with id {id} not found");
                }

                return lesson;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lesson by id: {ex.Message}");
                return null;
            }
        }

        public async Task<IEnumerable<Lesson>> GetByUserIdAsync(string id)
        {
            try
            {
                if (string.IsNullOrEmpty(id))
                {
                    throw new ArgumentNullException(nameof(id));
                }

                var lessons = await _context.Lessons
                    .Where(l => l.CreatedById == id)
                    .Include(l => l.CreatedBy)
                    .ToListAsync();

                return lessons;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lessons by user id: {ex.Message}");
                return Enumerable.Empty<Lesson>();
            }
        }

        public async Task<bool> ModifyAsync(Lesson lesson)
        {
            try
            {
                if (lesson == null)
                {
                    throw new ArgumentNullException(nameof(lesson));
                }

                var existingLesson = await _context.Lessons.FindAsync(lesson.Id);
                if (existingLesson == null)
                {
                    throw new KeyNotFoundException($"Lesson with id {lesson.Id} not found");
                }

                existingLesson.Title = lesson.Title;
                existingLesson.Description = lesson.Description;
                existingLesson.LessonFileUrl = lesson.LessonFileUrl;
                existingLesson.LessonType = lesson.LessonType;
                existingLesson.UpdatedDate = DateTime.UtcNow;

                _context.Lessons.Update(existingLesson);
                await _context.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error modifying lesson: {ex.Message}");
                return false;
            }
        }
    }
}