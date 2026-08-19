// bloom
// LessonAiService.cs
// Generates lesson content via Azure OpenAI, constrained to the LessonDto/LessonStepDto
// shape with a JSON schema so the response can be deserialized directly.

using System.Net.Http.Json;
using System.Text.Json;
using System.Text.Json.Serialization;
using bloom.Models.dto;

namespace bloom.Services
{
    public class LessonAiService : ILessonAiService
    {
        private readonly IHttpClientFactory _httpClientFactory;
        private readonly IRobotService _robotService;
        private readonly IConfiguration _config;
        private readonly ILogger<LessonAiService> _logger;

        // Fallback options shown when the Behaviors catalog is empty — mirrors the
        // defaultOptions in frontend/src/components/LessonStepBuilder.jsx (BEHAVIOR_FIELDS)
        // so AI suggestions line up with what the builder's dropdowns offer.
        private static readonly string[] DefaultBehaviorOptions = ["calm", "excited", "happy", "idle", "thinking"];
        private static readonly string[] DefaultFacialExpressionOptions = ["happy", "sad", "surprised", "neutral", "angry"];
        private static readonly string[] DefaultGazeOptions = ["forward", "left", "right", "up", "down"];
        private static readonly string[] DefaultHeadMovementOptions = ["nod", "shake", "tilt_left", "tilt_right", "still"];

        // visualAid and motorSequence are deliberately excluded — they reference uploaded
        // files / a curated motor-sequence library the model has no knowledge of.
        private static readonly string[] AllowedStepTypes =
        [
            "instruction", "introduction", "question", "wait_for_response",
            "feedback", "transition", "conclusion"
        ];

        private const int MaxSteps = 20;
        private const int MaxTimingSeconds = 300;

        private static readonly JsonSerializerOptions JsonOptions = new()
        {
            PropertyNameCaseInsensitive = true,
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            Converters = { new JsonStringEnumConverter() }
        };

        // Deliberately does not create the HttpClient or resolve Azure OpenAI config
        // here -- this service is constructor-injected into LessonController, which
        // also serves plain lesson CRUD endpoints that have nothing to do with AI, so
        // eagerly requiring Azure OpenAI config would 500 every lesson endpoint (not
        // just AI generation) whenever it isn't configured. Deferred to CallAsync,
        // where it only affects the AI generation endpoints that actually need it.
        public LessonAiService(IHttpClientFactory factory, IRobotService robotService, IConfiguration config, ILogger<LessonAiService> logger)
        {
            _httpClientFactory = factory;
            _robotService = robotService;
            _config = config;
            _logger = logger;
        }

        public async Task<LessonDto> GenerateLessonAsync(LessonAiGenerateRequestDto request)
        {
            var system = LessonSystemPrompt + "\n\n" + await BuildBehaviorOptionsSectionAsync();
            var userPrompt = BuildLessonUserPrompt(request);

            return await WithRepairRetryAsync(async errorFeedback =>
            {
                var prompt = errorFeedback is null
                    ? userPrompt
                    : $"{userPrompt}\n\nThe previous attempt was invalid: {errorFeedback}\nReturn a corrected lesson.";

                var json = await CallAsync(system, prompt, "lesson", LessonJsonSchema);
                var lesson = JsonSerializer.Deserialize<LessonDto>(json, JsonOptions)
                    ?? throw new InvalidOperationException("Empty lesson generation response.");
                return Sanitize(lesson);
            });
        }

        public async Task<LessonStepDto> GenerateStepAsync(LessonAiGenerateStepRequestDto request)
        {
            if (request.Lesson.Steps is null || request.StepIndex < 0 || request.StepIndex >= request.Lesson.Steps.Count)
                throw new ArgumentOutOfRangeException(nameof(request.StepIndex), "Step index is out of range for the supplied lesson.");

            var system = StepSystemPrompt + "\n\n" + await BuildBehaviorOptionsSectionAsync();
            var userPrompt = BuildStepUserPrompt(request);

            return await WithRepairRetryAsync(async errorFeedback =>
            {
                var prompt = errorFeedback is null
                    ? userPrompt
                    : $"{userPrompt}\n\nThe previous attempt was invalid: {errorFeedback}\nReturn a corrected step.";

                var json = await CallAsync(system, prompt, "lesson_step", StepJsonSchema);
                var step = JsonSerializer.Deserialize<LessonStepDto>(json, JsonOptions)
                    ?? throw new InvalidOperationException("Empty step generation response.");
                return SanitizeStep(step);
            });
        }

        // Tries once, and on any failure (HTTP, parse, or validation) retries a single time
        // with the error fed back to the model. Gives up and propagates after the second failure.
        private static async Task<T> WithRepairRetryAsync<T>(Func<string?, Task<T>> attempt)
        {
            try
            {
                return await attempt(null);
            }
            catch (Exception ex)
            {
                return await attempt(ex.Message);
            }
        }

        private async Task<string> CallAsync(string systemPrompt, string userPrompt, string schemaName, string schemaJson)
        {
            var deployment = _config["AZURE_OPENAI_DEPLOYMENT"]
                ?? throw new InvalidOperationException("AZURE_OPENAI_DEPLOYMENT is not configured.");
            var apiVersion = _config["AZURE_OPENAI_API_VERSION"] ?? "2024-08-01-preview";
            var http = _httpClientFactory.CreateClient("AzureOpenAILesson");

            var payload = new
            {
                messages = new[]
                {
                    new { role = "system", content = systemPrompt },
                    new { role = "user", content = userPrompt }
                },
                temperature = 0.7,
                response_format = new
                {
                    type = "json_schema",
                    json_schema = new
                    {
                        name = schemaName,
                        strict = true,
                        schema = JsonSerializer.Deserialize<JsonElement>(schemaJson)
                    }
                }
            };

            var url = $"openai/deployments/{deployment}/chat/completions?api-version={apiVersion}";
            using var response = await http.PostAsJsonAsync(url, payload);

            if (!response.IsSuccessStatusCode)
            {
                var body = await response.Content.ReadAsStringAsync();
                _logger.LogError("Azure OpenAI lesson generation failed ({Status}): {Body}", response.StatusCode, body);
                throw new InvalidOperationException("Lesson AI request failed.");
            }

            using var stream = await response.Content.ReadAsStreamAsync();
            using var doc = await JsonDocument.ParseAsync(stream);
            var content = doc.RootElement
                .GetProperty("choices")[0]
                .GetProperty("message")
                .GetProperty("content")
                .GetString();

            return string.IsNullOrWhiteSpace(content)
                ? throw new InvalidOperationException("Empty completion content.")
                : content;
        }

        private static LessonDto Sanitize(LessonDto lesson)
        {
            var steps = (lesson.Steps ?? []).Take(MaxSteps).Select(SanitizeStep).ToList();
            for (var i = 0; i < steps.Count; i++)
                steps[i].StepOrder = i + 1;

            return new LessonDto
            {
                Title = string.IsNullOrWhiteSpace(lesson.Title) ? "Untitled Lesson" : lesson.Title.Trim(),
                Description = string.IsNullOrWhiteSpace(lesson.Description) ? null : lesson.Description.Trim(),
                LessonType = lesson.LessonType,
                LearningObjectives = (lesson.LearningObjectives ?? [])
                    .Where(o => !string.IsNullOrWhiteSpace(o))
                    .Select(o => o!.Trim()),
                Steps = steps
            };
        }

        private static LessonStepDto SanitizeStep(LessonStepDto step)
        {
            if (string.IsNullOrWhiteSpace(step.Script))
                throw new InvalidOperationException("Generated step is missing a script.");

            return new LessonStepDto
            {
                Title = string.IsNullOrWhiteSpace(step.Title) ? null : step.Title.Trim(),
                Type = AllowedStepTypes.Contains(step.Type) ? step.Type : AllowedStepTypes[0],
                Script = step.Script.Trim(),
                TimingSeconds = step.TimingSeconds is >= 0 and <= MaxTimingSeconds ? step.TimingSeconds : null,
                Behaviors = step.Behaviors,
                Interaction = step.Interaction
            };
        }

        private const string LessonSystemPrompt =
            "You write lessons for Bloom, a robot used in pediatric speech and language therapy. " +
            "Output a single lesson matching the provided JSON schema. " +
            "Scripts are read aloud by the robot verbatim, so write them as short, simple, encouraging spoken lines, " +
            "one idea per step. Use only the listed step types and only fill behaviors/interaction fields that make " +
            "sense for that step's type — leave the rest null. Do not invent visual aid or motor sequence references.";

        private const string StepSystemPrompt =
            "You write a single step for an existing Bloom robot lesson (pediatric speech/language therapy), " +
            "matching the provided JSON schema. The script is read aloud by the robot verbatim — keep it short, " +
            "simple, and encouraging. Keep the step consistent with the lesson's title, objectives, and neighboring " +
            "steps. Use only the listed step types and only fill behaviors/interaction fields relevant to this " +
            "step's type. Do not invent visual aid or motor sequence references.";

        // Behavior options are curated per-deployment and change over time, so they're fetched
        // fresh on every call rather than baked into the static system prompt.
        private async Task<string> BuildBehaviorOptionsSectionAsync()
        {
            var behaviors = await _robotService.GetAvailableBehaviorsAsync();

            static string FormatOptions(IEnumerable<string?> fromCatalog, string[] defaults) =>
                string.Join(", ", defaults
                    .Concat(fromCatalog.Where(v => !string.IsNullOrWhiteSpace(v))!)
                    .Distinct(StringComparer.OrdinalIgnoreCase)
                    .OrderBy(v => v, StringComparer.OrdinalIgnoreCase));

            return
                "Available behavior options (prefer these exact values; leave a behaviors field null if none fit):\n" +
                $"- behavior: {FormatOptions(behaviors.Select(b => b.Name), DefaultBehaviorOptions)}\n" +
                $"- facial_expression: {FormatOptions(behaviors.Select(b => b.FacialExpression), DefaultFacialExpressionOptions)}\n" +
                $"- gaze: {FormatOptions(behaviors.Select(b => b.Gaze), DefaultGazeOptions)}\n" +
                $"- head_movement: {FormatOptions(behaviors.Select(b => b.HeadMovement), DefaultHeadMovementOptions)}";
        }

        private static string BuildLessonUserPrompt(LessonAiGenerateRequestDto request)
        {
            var prompt = $"Request: {request.Prompt}";

            if (request.ExistingLesson is not null)
            {
                prompt += "\n\nRevise within this existing in-progress lesson rather than starting over:\n" +
                    JsonSerializer.Serialize(request.ExistingLesson, JsonOptions);
            }

            return prompt;
        }

        private static string BuildStepUserPrompt(LessonAiGenerateStepRequestDto request)
        {
            var steps = request.Lesson.Steps!;
            var target = steps[request.StepIndex];

            var prompt =
                $"Lesson title: {request.Lesson.Title}\n" +
                $"Description: {request.Lesson.Description}\n" +
                $"Learning objectives: {JsonSerializer.Serialize(request.Lesson.LearningObjectives, JsonOptions)}\n" +
                $"All current steps in order: {JsonSerializer.Serialize(steps, JsonOptions)}\n" +
                $"Generate step {request.StepIndex + 1} of {steps.Count} (currently: {JsonSerializer.Serialize(target, JsonOptions)}).";

            if (!string.IsNullOrWhiteSpace(request.Instructions))
                prompt += $"\n\nInstructions: {request.Instructions}";

            return prompt;
        }

        private static readonly string StepJsonSchema = $$"""
        {
          "type": "object",
          "additionalProperties": false,
          "properties": {
            "title": { "type": ["string", "null"] },
            "type": { "type": "string", "enum": {{JsonSerializer.Serialize(AllowedStepTypes)}} },
            "script": { "type": "string" },
            "timingSeconds": { "type": ["integer", "null"] },
            "behaviors": {
              "type": ["object", "null"],
              "additionalProperties": false,
              "properties": {
                "behavior": { "type": ["string", "null"] },
                "facial_expression": { "type": ["string", "null"] },
                "gaze": { "type": ["string", "null"] },
                "head_movement": { "type": ["string", "null"] }
              },
              "required": ["behavior", "facial_expression", "gaze", "head_movement"]
            },
            "interaction": {
              "type": ["object", "null"],
              "additionalProperties": false,
              "properties": {
                "waitForResponse": { "type": "boolean" },
                "maxWaitSeconds": { "type": ["integer", "null"] },
                "correctAnswer": { "type": ["string", "null"] },
                "correctResponseScript": { "type": ["string", "null"] },
                "incorrectResponseScript": { "type": ["string", "null"] },
                "singleTurnLlm": { "type": "boolean" },
                "singleTurnLlmPrompt": { "type": ["string", "null"] },
                "llmFollowUp": { "type": "boolean" },
                "fallbackScript": { "type": ["string", "null"] }
              },
              "required": [
                "waitForResponse", "maxWaitSeconds", "correctAnswer", "correctResponseScript",
                "incorrectResponseScript", "singleTurnLlm", "singleTurnLlmPrompt", "llmFollowUp", "fallbackScript"
              ]
            }
          },
          "required": ["title", "type", "script", "timingSeconds", "behaviors", "interaction"]
        }
        """;

        private static readonly string LessonJsonSchema = $$"""
        {
          "type": "object",
          "additionalProperties": false,
          "properties": {
            "title": { "type": "string" },
            "description": { "type": ["string", "null"] },
            "lessonType": { "type": "string", "enum": ["Language", "Speech"] },
            "learningObjectives": { "type": "array", "items": { "type": "string" } },
            "steps": { "type": "array", "items": {{StepJsonSchema}} }
          },
          "required": ["title", "description", "lessonType", "learningObjectives", "steps"]
        }
        """;
    }
}
