// bloom
// IRsrService.cs
// Interface for AutoRSR microservice integration and assessment persistence.

using bloom.Models.dto;

namespace bloom.Services
{
    public interface IRsrService
    {
        Task<RsrAssessmentResultDto> AnalyzeAsync(
            IFormFile sessionAudio,
            int age,
            int percentile,
            string markersJson);

        Task<IEnumerable<RsrAssessmentSummaryDto>> GetAssessmentsAsync(string? pid = null);

        Task<RsrAssessmentResultDto?> GetAssessmentByIdAsync(Guid id);

        Task<RsrAssessmentResultDto?> GetAssessmentByPidAsync(string pid);
    }
}
