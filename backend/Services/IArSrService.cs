using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public interface IArSrService
    {
        Task<ArSrEnrollment> EnrollParticipantAsync(ArSrEnrollRequest request, string accountId);
        Task<ArSrEnrollment?> GetEnrollmentAsync(Guid enrollmentId);
        Task<List<ArSrEnrollment>> GetAllEnrollmentsAsync();
        Task<ArSrSession> CreateSessionAsync(ArSrSessionRequest request, ArSrMicroserviceResult analysisResult);
        Task<ArSrSession?> GetSessionAsync(Guid sessionId);
        Task<List<ArSrSession>> GetSessionsByEnrollmentAsync(Guid enrollmentId);
        Task<List<ArSrSession>> GetAllSessionsAsync();
    }
}
