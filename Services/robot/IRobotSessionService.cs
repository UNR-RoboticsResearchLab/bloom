
using bloom.Models;

namespace bloom.Services
{
    public interface IRobotSessionService
    {
        /// <summary>
        /// Starts a new robot session
        /// </summary>
        /// <param name="userId">ID of the user creating the session (null for anonymous sessions)</param>
        /// <param name="anon">Whether to create an anonymous session (overrides userId)</param>
        /// <returns>The created RobotSession</returns>
        Task<RobotSession> StartSessionAsync(string? userId = null, bool anon = false);

        /// <summary>
        /// Ends a robot session and archives remaining states
        /// </summary>
        /// <param name="sessionId">ID of the session to end</param>
        Task EndSessionAsync(Guid sessionId);

        /// <summary>
        /// Adds a robot to an active session with its initial state
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="robotState">Initial state of the robot</param>
        Task AddRobotToSessionAsync(Guid sessionId, RobotState robotState);

        /// <summary>
        /// Updates a robot's state within a session. Automatically archives to database every 10th change.
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="robotId">ID of the robot</param>
        /// <param name="newState">The new state of the robot</param>
        Task UpdateRobotStateAsync(Guid sessionId, Guid robotId, RobotState newState);

        /// <summary>
        /// Removes a robot from an active session (e.g., on disconnect/unresponsive)
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="robotId">ID of the robot to remove</param>
        Task RemoveRobotFromSessionAsync(Guid sessionId, Guid robotId);

        /// <summary>
        /// Gets all currently active robot states in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of active RobotStates</returns>
        Task<IEnumerable<RobotState>> GetCurrentStatesAsync(Guid sessionId);

        /// <summary>
        /// Gets all robot IDs currently active in a session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of active robot IDs</returns>
        Task<IEnumerable<Guid>> GetSessionRobotsAsync(Guid sessionId);

        /// <summary>
        /// Gets historical state snapshots for a session (for analysis)
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of historical state snapshots, ordered by timestamp descending</returns>
        Task<IEnumerable<RobotStateHistory>> GetSessionHistoryAsync(Guid sessionId);

        /// <summary>
        /// Retrieves a specific session with its metadata
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>The RobotSession or null if not found</returns>
        Task<RobotSession?> GetSessionAsync(Guid sessionId);

        /// <summary>
        /// Retrieves all robot sessions ordered by creation date (newest first)
        /// </summary>
        /// <returns>Collection of all RobotSessions</returns>
        Task<IEnumerable<RobotSession>> GetAllSessionsAsync();

        /// <summary>
        /// Generates and assigns a unique 6-digit session code to a session
        /// </summary>
        /// <returns>The generated session code</returns>
        Task<string> GenerateSessionCodeAsync();

        /// <summary>
        /// Gets a session by its session code
        /// </summary>
        /// <param name="code">The 6-digit session code</param>
        /// <returns>The RobotSession or null if not found</returns>
        Task<RobotSession?> GetSessionByCodeAsync(string code);
    }
}