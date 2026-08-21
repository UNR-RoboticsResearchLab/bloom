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

        public async Task<Lesson?> CreateAsync(LessonDto lesson)
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
                IEnumerable<string?> objectives = lesson.LearningObjectives;

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
                    IsPublic = lesson.IsPublic,
                    AdaptedFromLessonId = Guid.TryParse(lesson.AdaptedFromLessonId, out var adaptedFromId) ? adaptedFromId : null,
                    LearningObjectives = objectives?.ToList() ?? [],
                    TotalSteps = steps?.Count ?? 0
                };

                if (steps != null && steps.Count > 0)
                {
                    newLesson.Steps = await ResolveStepsAsync(steps);
                }

                _context.Lessons.Add(newLesson);
                await _context.SaveChangesAsync();
                return newLesson;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error creating lesson: {ex.Message}");
                return null;
            }
        }

        // Builds LessonStep entities from step DTOs, resolving/creating shared Behavior rows
        // as it goes (matched by name + facial expression). Shared by CreateAsync and
        // UpdateAsync so the two don't drift. Returned steps have no LessonId set -- assign
        // the result to a Lesson's Steps navigation and EF fixes up the foreign key.
        private async Task<List<LessonStep>> ResolveStepsAsync(List<LessonStepDto> steps)
        {
            var behaviorCache = new Dictionary<string, Behavior>(StringComparer.OrdinalIgnoreCase);
            var resolvedSteps = new List<LessonStep>();

            for (int i = 0; i < steps.Count; i++)
            {
                var s = steps[i];

                Behavior? behavior = null;
                if (s.Behaviors != null && !string.IsNullOrWhiteSpace(s.Behaviors.Behavior))
                {
                    var cacheKey = $"{s.Behaviors.Behavior}|{s.Behaviors.FacialExpression}";
                    if (!behaviorCache.TryGetValue(cacheKey, out behavior))
                    {
                        behavior = await _context.Behaviors.FirstOrDefaultAsync(b =>
                            b.Name == s.Behaviors.Behavior && b.FacialExpression == s.Behaviors.FacialExpression);
                        if (behavior == null)
                        {
                            behavior = new Behavior
                            {
                                Name = s.Behaviors.Behavior,
                                FacialExpression = s.Behaviors.FacialExpression,
                                Gaze = s.Behaviors.Gaze,
                                HeadMovement = s.Behaviors.HeadMovement,
                                CreatedDate = DateTime.UtcNow
                            };
                            _context.Behaviors.Add(behavior);
                        }
                        behaviorCache[cacheKey] = behavior;
                    }
                }

                var step = new LessonStep
                {
                    StepOrder = s.StepOrder > 0 ? s.StepOrder : i + 1,
                    Title = s.Title,
                    Type = s.Type,
                    Script = s.Script,
                    TimingSeconds = s.TimingSeconds,
                    VisualAid = s.VisualAid,
                    VisualAidLabels = s.VisualAidLabels,
                    VisualAidFooters = s.VisualAidFooters,
                    MotorSequence = s.MotorSequence,
                    Behaviors = behavior,
                };

                if (s.Interaction != null)
                {
                    step.Interaction = new StepInteraction
                    {
                        WaitForResponse = s.Interaction.WaitForResponse,
                        MaxWaitSeconds = s.Interaction.MaxWaitSeconds,
                        CorrectAnswer = s.Interaction.CorrectAnswer,
                        CorrectResponseScript = s.Interaction.CorrectResponseScript,
                        IncorrectResponseScript = s.Interaction.IncorrectResponseScript,
                        SingleTurnLlm = s.Interaction.SingleTurnLlm,
                        SingleTurnLlmPrompt = s.Interaction.SingleTurnLlmPrompt,
                        LlmFollowUp = s.Interaction.LlmFollowUp,
                        FallbackScript = s.Interaction.FallbackScript,
                        FallbackVisualAid = s.Interaction.FallbackVisualAid,
                        FallbackVisualAidLabels = s.Interaction.FallbackVisualAidLabels,
                    };
                }

                resolvedSteps.Add(step);
            }

            return resolvedSteps;
        }

        public async Task<Lesson?> GetByIdAsync(string id)
        {
            try
            {
                if (string.IsNullOrEmpty(id))
                    throw new ArgumentNullException(nameof(id));

                var lesson = await _context.Lessons
                    .Include(l => l.CreatedBy)
                    .Include(l => l.Assignments)
                    .Include(l => l.Steps.OrderBy(s => s.StepOrder))
                        .ThenInclude(s => s.Interaction)
                    .Include(l => l.Steps)
                        .ThenInclude(s => s.Behaviors)
                    .AsSplitQuery()
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

                var result = await _context.Lessons
                    .Where(l => l.CreatedBy != null && l.CreatedBy.Email == email)
                    .ToListAsync();

                if (result == null)
                {
                    throw new KeyNotFoundException("The Lesson could not be found for user" + email);
                }
                return result;
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

        // Replaces title/description/type/objectives and the entire step list. Steps are
        // fully replaced rather than diffed against existing ones -- the lesson builder
        // always submits its complete current draft, so there's no partial-update case to
        // support, and replacing avoids having to reconcile step identity across edits.
        public async Task<bool> UpdateAsync(LessonDto lesson)
        {
            try
            {
                ArgumentNullException.ThrowIfNull(lesson);

                if (string.IsNullOrEmpty(lesson.Id) || !Guid.TryParse(lesson.Id, out var lessonId))
                    throw new ArgumentException("A valid lesson Id is required.");

                var existing = await _context.Lessons
                    .Include(l => l.Steps)
                    .FirstOrDefaultAsync(l => l.Id == lessonId)
                    ?? throw new KeyNotFoundException($"Lesson with id {lesson.Id} not found");

                var steps = lesson.Steps;
                IEnumerable<string?> objectives = lesson.LearningObjectives;

                if ((steps == null || steps.Count == 0) && !string.IsNullOrEmpty(lesson.LessonDescription))
                {
                    (steps, objectives) = ParseLessonJson(lesson.LessonDescription);
                }

                existing.Title = lesson.Title;
                existing.Description = lesson.Description;
                existing.LessonType = lesson.LessonType;
                existing.IsPublic = lesson.IsPublic;
                existing.LearningObjectives = objectives?.ToList() ?? [];
                existing.UpdatedDate = DateTime.UtcNow;

                // Snapshot before removing -- RemoveRange'ing existing.Steps directly would
                // mutate that same navigation collection while iterating it (each removal's
                // relationship fixup drops the item from .Steps mid-enumeration), which
                // corrupts the change tracker and produces spurious concurrency exceptions.
                var oldSteps = existing.Steps.ToList();
                _context.LessonSteps.RemoveRange(oldSteps);
                existing.Steps.Clear();

                var resolvedSteps = steps != null && steps.Count > 0
                    ? await ResolveStepsAsync(steps)
                    : [];
                foreach (var step in resolvedSteps)
                    existing.Steps.Add(step);
                existing.TotalSteps = resolvedSteps.Count;

                await _context.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error updating lesson: {ex.Message}");
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
        private (List<LessonStepDto> steps, IEnumerable<string?> objectives) ParseLessonJson(string json)
        {
            var steps = new List<LessonStepDto>();
            IEnumerable<string?> objectives = [];

            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;

                // Extract objectives — check both known formats
                if (root.TryGetProperty("lesson", out var lessonMeta) &&
                    lessonMeta.TryGetProperty("objectives", out var obj1))
                {
                    objectives = obj1.EnumerateArray().Select(e => e.GetString());
                }
                else if (root.TryGetProperty("learning_objectives", out var obj2))
                {
                    objectives = obj2.EnumerateArray().Select(e => e.GetString());
                }

                if (!root.TryGetProperty("sequence", out var sequence))
                    return (steps, objectives);

                int order = 1;
                foreach (var step in sequence.EnumerateArray())
                {
                    StepInteractionDto? interactionDto = null;
                    if (step.TryGetProperty("interaction", out var inter))
                    {
                        interactionDto = new StepInteractionDto
                        {
                            WaitForResponse = inter.TryGetProperty("wait_for_response", out var wfr) && wfr.GetBoolean(),
                            MaxWaitSeconds = inter.TryGetProperty("max_wait_seconds", out var mws) ? mws.GetInt32() : null,
                            CorrectAnswer = inter.TryGetProperty("correct_answer", out var ca) ? ca.GetString() : null,
                            CorrectResponseScript = inter.TryGetProperty("correct_response_script", out var crs) ? crs.GetString() : null,
                            IncorrectResponseScript = inter.TryGetProperty("incorrect_response_script", out var irs) ? irs.GetString() : null,
                            SingleTurnLlm = inter.TryGetProperty("single_turn_llm", out var stl) && stl.GetBoolean(),
                            SingleTurnLlmPrompt = inter.TryGetProperty("single_turn_llm_prompt", out var stlp) ? stlp.GetString() : null,
                            LlmFollowUp = inter.TryGetProperty("llm_follow_up", out var lfu) && lfu.GetBoolean(),
                            FallbackScript = inter.TryGetProperty("fallback_script", out var fs) ? fs.GetString() : null,
                            FallbackVisualAid = inter.TryGetProperty("fallback_visual_aid", out var fva) ? fva.GetRawText() : null,
                            FallbackVisualAidLabels = inter.TryGetProperty("fallback_visual_aid_labels", out var fval) ? fval.GetRawText() : null,
                        };
                    }

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
                        VisualAidLabels = step.TryGetProperty("visual_aid_labels", out var valEl)
                            ? valEl.GetRawText() : null,
                        VisualAidFooters = step.TryGetProperty("visual_aid_footers", out var vafEl)
                            ? vafEl.GetRawText() : null,
                        MotorSequence = step.TryGetProperty("motor_sequence", out var msEl)
                            ? msEl.GetString() : null,
                        Behaviors = step.TryGetProperty("behaviors", out var beh) &&
                                    beh.TryGetProperty("behavior", out var bn) &&
                                    !string.IsNullOrWhiteSpace(bn.GetString())
                                    ? new StepBehaviorsDto
                                    {
                                        Behavior = bn.GetString()!,
                                        FacialExpression = beh.TryGetProperty("facial_expression", out var fe) ? fe.GetString() : null,
                                    }
                                    : null,
                        Interaction = interactionDto
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