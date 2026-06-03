using System.Text.Json;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class ArSrService : IArSrService
    {
        private readonly ArSrDbContext _db;
        private readonly BloomDbContext _bloomDb;

        public ArSrService(ArSrDbContext db, BloomDbContext bloomDb)
        {
            _db = db;
            _bloomDb = bloomDb;
        }

        public async Task<ArSrEnrollment> EnrollParticipantAsync(ArSrEnrollRequest request, string accountId)
        {
            var enrollment = new ArSrEnrollment
            {
                ParticipantAccountId  = accountId,
                Condition             = request.Condition,
                Gender                = request.Gender,
                Ethnicity             = request.Ethnicity,
                NativeLanguage        = request.NativeLanguage,
                HearingLoss           = request.HearingLoss,
                ReceivingSpeechTherapy = request.ReceivingSpeechTherapy,
                DemographicNotes      = request.DemographicNotes,
            };

            _db.Enrollments.Add(enrollment);
            await _db.SaveChangesAsync();
            return enrollment;
        }

        public async Task<ArSrEnrollment?> GetEnrollmentAsync(Guid enrollmentId) =>
            await _db.Enrollments
                .Include(e => e.Sessions)
                .FirstOrDefaultAsync(e => e.Id == enrollmentId);

        public async Task<List<ArSrEnrollment>> GetAllEnrollmentsAsync() =>
            await _db.Enrollments
                .Include(e => e.Sessions)
                .OrderByDescending(e => e.EnrolledAt)
                .ToListAsync();

        public async Task<ArSrSession> CreateSessionAsync(
            ArSrSessionRequest request,
            ArSrMicroserviceResult analysis)
        {
            var session = new ArSrSession
            {
                EnrollmentId       = request.EnrollmentId,
                AgeMonths          = request.AgeMonths,
                Percentile         = request.Percentile,
                TotalScore         = analysis.TotalScore,
                Result             = analysis.Result,
                SessionFolder      = analysis.SessionFolder,
                VideoFilePath      = request.VideoFilePath,
                AdministratorNotes = request.AdministratorNotes,
            };

            foreach (var s in analysis.Sentences)
            {
                if (!int.TryParse(s.Id, out int num)) continue;
                session.SentenceResults.Add(new ArSrSentenceResult
                {
                    SentenceNumber = num,
                    GroundTruth    = s.GroundTruth,
                    Response       = s.Response,
                    Errors         = s.Errors,
                    Score          = s.Score,
                    EditScript     = s.EditScript != null
                        ? JsonSerializer.Serialize(s.EditScript)
                        : null,
                });
            }

            foreach (var (sentenceIdStr, path) in analysis.RecordingPaths)
            {
                if (!int.TryParse(sentenceIdStr, out int num)) continue;
                session.Recordings.Add(new ArSrRecording
                {
                    SentenceNumber = num,
                    FilePath       = path,
                });
            }

            _db.Sessions.Add(session);
            await _db.SaveChangesAsync();
            return session;
        }

        public async Task<ArSrSession?> GetSessionAsync(Guid sessionId) =>
            await _db.Sessions
                .Include(s => s.SentenceResults)
                .Include(s => s.Recordings)
                .FirstOrDefaultAsync(s => s.Id == sessionId);

        public async Task<List<ArSrSession>> GetSessionsByEnrollmentAsync(Guid enrollmentId) =>
            await _db.Sessions
                .Include(s => s.SentenceResults)
                .Where(s => s.EnrollmentId == enrollmentId)
                .OrderByDescending(s => s.CreatedAt)
                .ToListAsync();

        public async Task<List<ArSrSession>> GetAllSessionsAsync() =>
            await _db.Sessions
                .Include(s => s.Enrollment)
                .Include(s => s.SentenceResults)
                .OrderByDescending(s => s.CreatedAt)
                .ToListAsync();
    }
}
