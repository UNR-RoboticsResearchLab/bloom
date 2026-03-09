

using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class LessonService : ILessonService
    {
        private readonly BloomDbContext _context;
        private readonly IWebHostEnvironment _env;

        public LessonService(BloomDbContext dbContext, IWebHostEnvironment env)
        {
            _context = dbContext;
            _env = env;
        }

        public async Task<bool> CreateAsync(LessonDto lesson)
        {
            try
            {
                ArgumentNullException.ThrowIfNull(lesson);

                lesson.CreatedDate = DateTime.UtcNow;

                // Create lesson file from LessonDescription
                var lessonsDir = Path.Combine(_env.ContentRootPath, "lessons");
                Directory.CreateDirectory(lessonsDir);
                var fileName = $"{Guid.NewGuid()}.json";
                var filePath = Path.Combine(lessonsDir, fileName);
                await File.WriteAllTextAsync(filePath, lesson.LessonDescription ?? string.Empty);

                var account = await _context.Accounts.FindAsync(lesson.CreatedById)
                    ?? throw new KeyNotFoundException($"Account with id {lesson.CreatedById} not found");

                var newLesson = new Lesson
                {
                    Title = lesson.Title,
                    Description = lesson.Description,
                    CreatedDate = lesson.CreatedDate,
                    LessonFileUrl = filePath,
                    LessonType = lesson.LessonType,
                    CreatedById = lesson.CreatedById,
                    CreatedBy = account
                };

                _context.Lessons.Add(newLesson);
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
                    .Where(l => l.CreatedBy != null && l.CreatedBy.Email == email)
                    .ToListAsync();

                return lessons;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lessons by email: {ex.Message}");
                return Enumerable.Empty<Lesson>();
            }
        }

        public async Task<Lesson?> GetByIdAsync(string id)
        {

            try
            {
                if (string.IsNullOrEmpty(id))
                {
                    throw new ArgumentNullException(nameof(id));
                }

                var guid = Guid.Parse(id);

                var lesson = await _context.Lessons
                    .Include(l => l.CreatedBy)
                    .Include(l => l.Assignments)
                    .FirstOrDefaultAsync(l => l.Id == guid);

                if (lesson == null)
                {
                    throw new KeyNotFoundException($"Lesson with id {id} not found");
                }

                // Get and parse lesson file to verify it's valid JSON
                var filePath = lesson.LessonFileUrl;

                // If the path is relative, resolve it against the content root
                if (!Path.IsPathRooted(filePath))
                {
                    filePath = Path.Combine(_env.ContentRootPath, filePath);
                }

                // var lessonContent = await File.ReadAllTextAsync(filePath);
                // try
                // {
                //     var parsedJson = System.Text.Json.JsonDocument.Parse(lessonContent);
                // }
                // catch (System.Text.Json.JsonException)
                // {
                //     throw new ArgumentException("LessonFileUrl must point to a valid JSON file.");
                // }

                if (!File.Exists(filePath))
                {
                    Console.WriteLine($"Lesson file missing in container: {filePath}");
                    return lesson;
                }

                var lessonContent = await File.ReadAllTextAsync(filePath);

                return lesson;

            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lesson by id: {ex.Message}");
                return null;
            }
        }

        public async Task<IEnumerable<Lesson>> GetAllAsync()
        {
            try
            {
                var lessons = await _context.Lessons
                    .Include(l => l.CreatedBy)
                    .ToListAsync();

                return lessons;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting all lessons: {ex.Message}");
                return Enumerable.Empty<Lesson>();
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