// bloom
// ISLPClientService.cs
// Interface for managing SLPClient records (student-SLP associations).

using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public enum AddTeacherResult
    {
        Added,
        ClientNotFound,
        TeacherNotFound,
        AlreadyAssociated
    }

    public enum DeleteClientResult
    {
        Deleted,
        NotFound,
        Forbidden
    }

    public interface ISLPClientService
    {
        /// <summary>
        /// Create a new SLPClient linking a student to the requesting SLP.
        /// </summary>
        Task<SLPClient> CreateAsync(string name, string slpId, string studentId);

        /// <summary>
        /// Create a new SLPClient and return the response DTO. Returns null if the student or SLP is missing.
        /// </summary>
        Task<SLPClientResponseDto?> CreateClientAsync(string name, string slpId, string studentId);

        /// <summary>
        /// List all SLPClients where the given SLP is one of the teachers.
        /// </summary>
        Task<List<SLPClientResponseDto>> GetClientsForSlpAsync(string slpId);

        /// <summary>
        /// Get a single SLPClient with assignments and progress, or null if not found.
        /// </summary>
        Task<SLPClientResponseDto?> GetClientAsync(Guid id);

        /// <summary>
        /// Add another SLP as a teacher on an existing client.
        /// </summary>
        Task<AddTeacherResult> AddTeacherAsync(Guid clientId, string teacherId);

        /// <summary>
        /// Delete an SLPClient. The requesting SLP must be one of the client's teachers.
        /// </summary>
        Task<DeleteClientResult> DeleteClientAsync(Guid clientId, string slpId);
    }
}
