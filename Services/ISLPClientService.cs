// bloom
// ISLPClientService.cs
// Interface for managing SLPClient records (student-SLP associations).

using bloom.Models;

namespace bloom.Services
{
    public interface ISLPClientService
    {
        /// <summary>
        /// Create a new SLPClient linking a student to the requesting SLP.
        /// </summary>
        Task<SLPClient> CreateAsync(string name, string slpId, string studentId);
    }
}
