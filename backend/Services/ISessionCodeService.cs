// bloom
// ISessionCodeService.cs
// Interface for session code generation and management
// Created: 11/21/2025

namespace bloom.Services
{
    public interface ISessionCodeService
    {
        /// <summary>
        /// Generates a unique 6-digit session code
        /// </summary>
        /// <returns>A 6-digit code as a string</returns>
        string GenerateSessionCode();

        /// <summary>
        /// Checks if a session code already exists
        /// </summary>
        /// <param name="code">The session code to check</param>
        /// <returns>True if the code exists, false otherwise</returns>
        Task<bool> CodeExistsAsync(string code);

        /// <summary>
        /// Gets a session ID by its session code
        /// </summary>
        /// <param name="code">The session code</param>
        /// <returns>The session ID or null if not found</returns>
        Task<Guid?> GetSessionIdByCodeAsync(string code);
    }
}
