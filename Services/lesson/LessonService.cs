using System.Text.Json;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
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

        public async Task<bool> CreateAsync(LessonDto lesson)
        {
            try
            {
                ArgumentNullException.ThrowIfNull(lesson);

                if (string.IsNullOrEmpty(lesson.CreatedById))
                    throw new ArgumentException("CreatedById is required.");

                var account = await _context.Accounts.FindAsync(lesson.CreatedById)
                    ?? throw new KeyNotFoundException($"Account with id {lesson.CreatedById} not found");

                // Resolve steps: prefer structured Steps, fall back to parsing LessonDescription JSON
                var steps = lesson.Steps;
                string? objectives = lesson.LearningObjectives;

                if ((steps == null || steps.Count == 0) && !string.IsNullOrEmpty(lesson.LessonDescription))
                {
                    (steps, objectives) = ParseLessonJson(lesson.LessonDescription);
                }

                var newLesson = new Lesson
                {
                    Title = lesson.Title,
                    Description = lesson.Description,
                    CreatedDate = DateTime.UtcNow,
                    LessonType = lesson.LessonType,
                    CreatedById = lesson.CreatedById,
                    CreatedBy = account,
                    LearningObjectives = objectives,
                    TotalSteps = steps?.Count ?? 0
                };

                if (steps != null && steps.Count > 0)
                {
                    newLesson.Steps = steps.Select((s, i) => new LessonStep
                    {
                        LessonId = newLesson.Id,
                        StepOrder = s.StepOrder > 0 ? s.StepOrder : i + 1,
                        Type = s.Type,
                        Script = s.Script,
                        TimingSeconds = s.TimingSeconds,
                        VisualAid = s.VisualAid,
                        Behaviors = s.Behaviors,
                        Interaction = s.Interaction
                    }).ToList();
                }

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

        public async Task<Lesson> GetByIdAsync(string id)
        {
            try
            {
                if (string.IsNullOrEmpty(id))
                    throw new ArgumentNullException(nameof(id));

                var lesson = await _context.Lessons
                    .Include(l => l.CreatedBy)
                    .Include(l => l.Assignments)
                    .Include(l => l.Steps.OrderBy(s => s.StepOrder))
                    .FirstOrDefaultAsync(l => l.Id == new Guid(id));

                if (lesson == null)
                    throw new KeyNotFoundException($"Lesson with id {id} not found");

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
                    throw new ArgumentNullException(nameof(id));

                return await _context.Lessons
                    .Where(l => l.CreatedById == id)
                    .Include(l => l.CreatedBy)
                    .ToListAsync();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lessons by user id: {ex.Message}");
                return Enumerable.Empty<Lesson>();
            }
        }

        public async Task<IEnumerable<Lesson>> GetByEmailAsync(string email)
        {
            try
            {
                if (string.IsNullOrEmpty(email))
                    throw new ArgumentNullException(nameof(email));

                return await _context.Lessons
                    .Where(l => l.CreatedBy.Email == email)
                    .ToListAsync();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting lessons by email: {ex.Message}");
                return Enumerable.Empty<Lesson>();
            }
        }

        public async Task<bool> ModifyAsync(Lesson lesson)
        {
            try
            {
                ArgumentNullException.ThrowIfNull(lesson);

                var existing = await _context.Lessons.FindAsync(lesson.Id)
                    ?? throw new KeyNotFoundException($"Lesson with id {lesson.Id} not found");

                existing.Title = lesson.Title;
                existing.Description = lesson.Description;
                existing.UpdatedDate = DateTime.UtcNow;

                _context.Lessons.Update(existing);
                await _context.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error modifying lesson: {ex.Message}");
                return false;
            }
        }

        public async Task DeleteByIdAsync(string id)
        {
            try
            {
                if (string.IsNullOrEmpty(id))
                    throw new ArgumentNullException(nameof(id));

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

        public async Task<bool> RemoveStepAsync(Guid lessonId, Guid stepId)
        {
            try
            {
                var step = await _context.LessonSteps
                    .FirstOrDefaultAsync(s => s.Id == stepId && s.LessonId == lessonId);

                if (step == null)
                    return false;

                _context.LessonSteps.Remove(step);

                var lesson = await _context.Lessons.FindAsync(lessonId);
                if (lesson != null)
                    lesson.TotalSteps = Math.Max(0, lesson.TotalSteps - 1);

                await _context.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error removing step: {ex.Message}");
                return false;
            }
        }

        // Parses the original lesson JSON file format into structured steps and objectives.
        // Handles both { lesson: { objectives: [...] }, sequence: [...] } and
        // { learning_objectives: [...], sequence: [...] } formats.
        private (List<LessonStepDto> steps, string? objectives) ParseLessonJson(string json)
        {
            var steps = new List<LessonStepDto>();
            string? objectives = null;

            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;

                // Extract objectives — check both known formats
                if (root.TryGetProperty("lesson", out var lessonMeta) &&
                    lessonMeta.TryGetProperty("objectives", out var obj1))
                {
                    objectives = obj1.GetRawText();
                }
                else if (root.TryGetProperty("learning_objectives", out var obj2))
                {
                    objectives = obj2.GetRawText();
                }

                if (!root.TryGetProperty("sequence", out var sequence))
                    return (steps, objectives);

                int order = 1;
                foreach (var step in sequence.EnumerateArray())
                {
                    steps.Add(new LessonStepDto
                    {
                        StepOrder = step.TryGetProperty("id", out var idProp) && idProp.ValueKind == JsonValueKind.Number
                            ? idProp.GetInt32() : order,
                        Type = step.TryGetProperty("type", out var typeProp)
                            ? typeProp.GetString() ?? "unknown" : "unknown",
                        Script = step.TryGetProperty("script", out var scriptProp)
                            ? scriptProp.GetString() ?? "" : "",
                        TimingSeconds = step.TryGetProperty("timing_seconds", out var tp) && tp.ValueKind == JsonValueKind.Number
                            ? tp.GetInt32() : null,
                        VisualAid = step.TryGetProperty("visual_aid", out var va)
                            ? va.GetRawText() : null,
                        Behaviors = step.TryGetProperty("behaviors", out var beh)
                            ? beh.GetRawText() : null,
                        Interaction = step.TryGetProperty("interaction", out var inter)
                            ? inter.GetRawText() : null
                    });
                    order++;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error parsing lesson JSON: {ex.Message}");
            }

            return (steps, objectives);
        }
    }
}