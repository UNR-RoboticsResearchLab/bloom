// bloom
// RsrService.cs
// Forwards RSR assessment requests to the AutoRSR Python microservice,
// then persists and retrieves results from the database.

using System.Text.Json;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class RsrService : IRsrService
    {
        private readonly HttpClient _http;
        private readonly BloomDbContext _db;
        private readonly ILogger<RsrService> _logger;

        public RsrService(IHttpClientFactory httpClientFactory, BloomDbContext db, ILogger<RsrService> logger)
        {
            _http = httpClientFactory.CreateClient("AutoRsr");
            _db = db;
            _logger = logger;
        }

        public async Task<RsrAssessmentResultDto> AnalyzeAsync(
            IFormFile sessionAudio,
            int age,
            int percentile,
            string markersJson)
        {
            using var form = new MultipartFormDataContent();
            using var audioStream = sessionAudio.OpenReadStream();
            var audioContent = new StreamContent(audioStream);
            audioContent.Headers.ContentType = new System.Net.Http.Headers.MediaTypeHeaderValue(
                sessionAudio.ContentType ?? "audio/webm");

            form.Add(audioContent, "session_audio", sessionAudio.FileName);
            form.Add(new StringContent(age.ToString()), "age");
            form.Add(new StringContent(percentile.ToString()), "percentile");
            form.Add(new StringContent(markersJson), "markers");

            var response = await _http.PostAsync("/api/analyze", form);
            response.EnsureSuccessStatusCode();

            var responseBody = await response.Content.ReadAsStringAsync();

            using var doc = JsonDocument.Parse(responseBody);
            var root = doc.RootElement;

            int totalScore = root.GetProperty("totalScore").GetInt32();
            string decision = root.GetProperty("result").GetString() ?? "N/A";

            var sentences = root.GetProperty("sentences").EnumerateArray()
                .Select(s => new RsrSentenceResultDto
                {
                    Id = s.GetProperty("id").GetString() ?? "",
                    GroundTruth = s.GetProperty("groundTruth").GetString() ?? "",
                    Response = s.GetProperty("response").GetString() ?? "",
                    Errors = s.GetProperty("errors").GetInt32(),
                    Score = s.GetProperty("score").GetInt32(),
                })
                .ToList();

            var pid = GeneratePid();

            var assessment = new RsrAssessment
            {
                Pid = pid,
                AgeInMonths = age,
                Percentile = percentile,
                TotalScore = totalScore,
                Decision = decision,
                ResultJson = responseBody,
                CreatedAt = DateTime.UtcNow,
            };

            _db.RsrAssessments.Add(assessment);
            await _db.SaveChangesAsync();

            return new RsrAssessmentResultDto
            {
                AssessmentId = assessment.Id,
                Pid = pid,
                TotalScore = totalScore,
                Decision = decision,
                Sentences = sentences,
                CreatedAt = assessment.CreatedAt,
            };
        }

        public async Task<IEnumerable<RsrAssessmentSummaryDto>> GetAssessmentsAsync(string? pid = null)
        {
            var query = _db.RsrAssessments.AsQueryable();

            if (pid != null)
                query = query.Where(a => a.Pid == pid);

            return await query
                .OrderByDescending(a => a.CreatedAt)
                .Select(a => new RsrAssessmentSummaryDto
                {
                    Id = a.Id,
                    Pid = a.Pid,
                    AgeInMonths = a.AgeInMonths,
                    Percentile = a.Percentile,
                    TotalScore = a.TotalScore,
                    Decision = a.Decision,
                    CreatedAt = a.CreatedAt,
                })
                .ToListAsync();
        }

        public async Task<RsrAssessmentResultDto?> GetAssessmentByIdAsync(Guid id)
        {
            var assessment = await _db.RsrAssessments.FindAsync(id);
            return assessment == null ? null : ToResultDto(assessment);
        }

        public async Task<RsrAssessmentResultDto?> GetAssessmentByPidAsync(string pid)
        {
            var assessment = await _db.RsrAssessments
                .Where(a => a.Pid == pid)
                .OrderByDescending(a => a.CreatedAt)
                .FirstOrDefaultAsync();
            return assessment == null ? null : ToResultDto(assessment);
        }

        private static RsrAssessmentResultDto ToResultDto(RsrAssessment assessment)
        {
            using var doc = JsonDocument.Parse(assessment.ResultJson);
            var root = doc.RootElement;

            var sentences = root.GetProperty("sentences").EnumerateArray()
                .Select(s => new RsrSentenceResultDto
                {
                    Id = s.GetProperty("id").GetString() ?? "",
                    GroundTruth = s.GetProperty("groundTruth").GetString() ?? "",
                    Response = s.GetProperty("response").GetString() ?? "",
                    Errors = s.GetProperty("errors").GetInt32(),
                    Score = s.GetProperty("score").GetInt32(),
                })
                .ToList();

            return new RsrAssessmentResultDto
            {
                AssessmentId = assessment.Id,
                Pid = assessment.Pid,
                TotalScore = assessment.TotalScore,
                Decision = assessment.Decision,
                Sentences = sentences,
                CreatedAt = assessment.CreatedAt,
            };
        }

        private static string GeneratePid()
        {
            const string chars = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
            var random = new Random();
            var code = new string(Enumerable.Range(0, 6).Select(_ => chars[random.Next(chars.Length)]).ToArray());
            return $"RSR-{code}";
        }
    }
}
