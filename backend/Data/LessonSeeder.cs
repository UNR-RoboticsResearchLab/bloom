// bloom
// LessonSeeder.cs
// Seeds the database with lessons from LessonFiles/lesson_description/*.json on startup.

using bloom.Models;
using Microsoft.AspNetCore.Identity;
using System.Text.Json;

namespace bloom.Data
{
    public static class LessonSeeder
    {
        private static readonly JsonSerializerOptions _jsonOptions = new() { PropertyNameCaseInsensitive = true };

        public static async Task SeedLessonsFromFilesAsync(BloomDbContext db, UserManager<Account> userManager)
        {
            var admin = await userManager.FindByEmailAsync("admin@example.com");
            if (admin == null)
            {
                Console.WriteLine("[LessonSeeder] Admin user not found, skipping lesson seed.");
                return;
            }

            var lessonDir = Path.Combine(AppContext.BaseDirectory, "LessonFiles", "lesson_description");
            if (!Directory.Exists(lessonDir))
            {
                Console.WriteLine($"[LessonSeeder] Lesson directory not found: {lessonDir}");
                return;
            }

            var files = Directory.GetFiles(lessonDir, "*.json");
            if (files.Length == 0)
            {
                Console.WriteLine("[LessonSeeder] No lesson JSON files found.");
                return;
            }

            // Pre-load existing behaviors keyed by "name|facialExpression" to avoid duplicates
            var behaviorCache = db.Behaviors.ToDictionary(
                b => $"{b.Name}|{b.FacialExpression}",
                StringComparer.OrdinalIgnoreCase);

            // Seed behaviors from all lesson files before processing lessons,
            // so the behavior table is populated even when lessons already exist.
            await SeedBehaviorsFromFilesAsync(db, files, behaviorCache);

            foreach (var file in files)
            {
                await SeedLessonFileAsync(db, admin.Id, file, behaviorCache);
            }

            await db.SaveChangesAsync();
        }

        private static async Task SeedBehaviorsFromFilesAsync(BloomDbContext db, string[] files, Dictionary<string, Behavior> behaviorCache)
        {
            foreach (var filePath in files)
            {
                string json;
                try { json = await File.ReadAllTextAsync(filePath); }
                catch (Exception ex)
                {
                    Console.WriteLine($"[LessonSeeder] Failed to read {filePath}: {ex.Message}");
                    continue;
                }

                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                if (!root.TryGetProperty("sequence", out var seqEl)) continue;

                foreach (var stepEl in seqEl.EnumerateArray())
                {
                    if (!stepEl.TryGetProperty("behaviors", out var behavEl)) continue;
                    var behName = behavEl.TryGetProperty("behavior", out var behNameEl) ? behNameEl.GetString() : null;
                    var facialExpr = behavEl.TryGetProperty("facial_expression", out var feEl) ? feEl.GetString() : null;
                    if (string.IsNullOrWhiteSpace(behName)) continue;

                    var cacheKey = $"{behName}|{facialExpr}";
                    if (!behaviorCache.ContainsKey(cacheKey))
                    {
                        var behavior = new Behavior { Name = behName, FacialExpression = facialExpr, CreatedDate = DateTime.UtcNow };
                        db.Behaviors.Add(behavior);
                        behaviorCache[cacheKey] = behavior;
                    }
                }
            }
        }

        private static async Task SeedLessonFileAsync(BloomDbContext db, string adminId, string filePath, Dictionary<string, Behavior> behaviorCache)
        {
            var fileName = Path.GetFileNameWithoutExtension(filePath);

            // Skip if a lesson with this title (from the file) already exists
            string json;
            try
            {
                json = await File.ReadAllTextAsync(filePath);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[LessonSeeder] Failed to read {filePath}: {ex.Message}");
                return;
            }

            using var doc = JsonDocument.Parse(json);
            var root = doc.RootElement;

            // Parse top-level lesson metadata
            var lessonMeta = root.TryGetProperty("lesson", out var metaEl) ? metaEl : (JsonElement?)null;
            var title = lessonMeta?.TryGetProperty("title", out var titleEl) == true ? titleEl.GetString() : fileName;
            var grade = lessonMeta?.TryGetProperty("grade", out var gradeEl) == true ? gradeEl.GetString() : null;
            IEnumerable<string?> objectives = lessonMeta?.TryGetProperty("objectives", out var objEl) == true
                ? objEl.EnumerateArray().Select(e => e.GetString()).ToList()
                : [];

            // Deduplicate by title
            if (db.Lessons.Any(l => l.Title == title))
            {
                Console.WriteLine($"[LessonSeeder] Lesson '{title}' already exists, skipping.");
                return;
            }

            // Parse sequence steps
            var steps = new List<LessonStep>();
            if (root.TryGetProperty("sequence", out var seqEl))
            {
                foreach (var stepEl in seqEl.EnumerateArray())
                {
                    var stepOrder = stepEl.TryGetProperty("id", out var idEl) ? idEl.GetInt32() : steps.Count + 1;
                    var type = stepEl.TryGetProperty("type", out var typeEl) ? typeEl.GetString() ?? "unknown" : "unknown";
                    var script = stepEl.TryGetProperty("script", out var scriptEl) ? scriptEl.GetString() ?? "" : "";
                    int? timingSeconds = stepEl.TryGetProperty("timing_seconds", out var timingEl) ? timingEl.GetInt32() : null;

                    string? visualAid = null;
                    if (stepEl.TryGetProperty("visual_aid", out var vaEl))
                        visualAid = vaEl.ValueKind == JsonValueKind.Array ? vaEl.GetRawText() : vaEl.GetString();


                    StepInteraction? interaction = null;
                    if (stepEl.TryGetProperty("interaction", out var interEl))
                    {
                        interaction = new StepInteraction
                        {
                            WaitForResponse = interEl.TryGetProperty("wait_for_response", out var wfr) && wfr.GetBoolean(),
                            MaxWaitSeconds = interEl.TryGetProperty("max_wait_seconds", out var mws) ? mws.GetInt32() : null,
                            CorrectAnswer = interEl.TryGetProperty("correct_answer", out var ca) ? ca.GetString() : null,
                            CorrectResponseScript = interEl.TryGetProperty("correct_response_script", out var crs) ? crs.GetString() : null,
                            IncorrectResponseScript = interEl.TryGetProperty("incorrect_response_script", out var irs) ? irs.GetString() : null,
                            SingleTurnLlm = interEl.TryGetProperty("single_turn_llm", out var stl) && stl.GetBoolean(),
                            SingleTurnLlmPrompt = interEl.TryGetProperty("single_turn_llm_prompt", out var stlp) ? stlp.GetString() : null,
                            LlmFollowUp = interEl.TryGetProperty("llm_follow_up", out var lfu) && lfu.GetBoolean(),
                            FallbackScript = interEl.TryGetProperty("fallback_script", out var fs) ? fs.GetString() : null,
                            FallbackVisualAid = interEl.TryGetProperty("fallback_visual_aid", out var fva) ? fva.GetRawText() : null,
                            FallbackVisualAidLabels = interEl.TryGetProperty("fallback_visual_aid_labels", out var fval) ? fval.GetRawText() : null,
                        };
                    }

                    string? visualAidLabels = stepEl.TryGetProperty("visual_aid_labels", out var valEl)
                        ? valEl.GetRawText() : null;
                    string? visualAidFooters = stepEl.TryGetProperty("visual_aid_footers", out var vafEl)
                        ? vafEl.GetRawText() : null;
                    string? motorSequence = stepEl.TryGetProperty("motor_sequence", out var msEl)
                        ? msEl.GetString() : null;

                    Behavior? stepBehavior = null;
                    if (stepEl.TryGetProperty("behaviors", out var behavEl))
                    {
                        var behName = behavEl.TryGetProperty("behavior", out var behNameEl) ? behNameEl.GetString() : null;
                        var facialExpr = behavEl.TryGetProperty("facial_expression", out var feEl) ? feEl.GetString() : null;

                        if (!string.IsNullOrWhiteSpace(behName))
                        {
                            var cacheKey = $"{behName}|{facialExpr}";
                            if (!behaviorCache.TryGetValue(cacheKey, out stepBehavior))
                            {
                                stepBehavior = new Behavior
                                {
                                    Name = behName,
                                    FacialExpression = facialExpr,
                                    CreatedDate = DateTime.UtcNow
                                };
                                db.Behaviors.Add(stepBehavior);
                                behaviorCache[cacheKey] = stepBehavior;
                            }
                        }
                    }

                    var step = new LessonStep
                    {
                        StepOrder = stepOrder,
                        Type = type,
                        Script = script,
                        TimingSeconds = timingSeconds,
                        VisualAid = visualAid,
                        VisualAidLabels = visualAidLabels,
                        VisualAidFooters = visualAidFooters,
                        MotorSequence = motorSequence,
                        Behaviors = stepBehavior,
                    };

                    if (interaction != null)
                    {
                        step.Interaction = interaction;
                    }

                    steps.Add(step);
                }
            }

            var lesson = new Lesson
            {
                Title = title ?? fileName,
                Description = grade != null ? $"Grade: {grade}" : null,
                CreatedDate = DateTime.UtcNow,
                CreatedById = adminId,
                LessonType = LessonType.Language,
                TotalSteps = steps.Count,
                LearningObjectives = objectives,
                Steps = steps
            };

            db.Lessons.Add(lesson);
            Console.WriteLine($"[LessonSeeder] Seeded lesson '{lesson.Title}' with {steps.Count} steps.");
        }
    }
}
