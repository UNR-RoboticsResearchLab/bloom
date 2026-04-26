
using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public interface IRobotSessionService
    {
        /// <summary>
        /// Starts a new robot session
        /// </summary>
        /// <param name="robotId"> ID of the robot making a session</param> 
        /// <param name="userId">ID of the user creating the session (null for anonymous sessions)</param>
        /// <param name="anon">Whether to create an anonymous session (overrides userId)</param>
        /// <returns>The created RobotSession</returns>
        Task<RobotSession> StartSessionAsync(Guid robotId, string? userId = null, bool anon = false);

        /// <summary>
        /// Ends a robot session and archives remaining states
        /// </summary>
        /// <param name="sessionId">ID of the session to end</param>
        Task EndSessionAsync(Guid sessionId);

        /// <summary>
        /// Adds a robot to an active session with its initial state
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="robotId">Id of the robot</param>
        Task AddRobotToSessionAsync(Guid sessionId, Guid robotId);

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

        /// <summary>
        /// Logs a student interaction during a lesson in a session
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <param name="dto">Interaction details</param>
        /// <returns>ID of the created interaction record</returns>
        Task<Guid> LogLessonInteractionAsync(Guid sessionId, LogLessonInteractionDto dto);

        /// <summary>
        /// Creates or updates lesson progress for the student in a session
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <param name="dto">Progress update data</param>
        Task UpdateLessonProgressAsync(Guid sessionId, UpdateLessonProgressDto dto);

        /// <summary>
        /// Gets a pending lesson for the session if one is assigned
        /// Returns the full lesson JSON along with metadata for the robot to execute
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <returns>Pending lesson data or null if none exists</returns>
        Task<dynamic?> GetPendingLessonAsync(Guid sessionId);

        /// <summary>
        /// Records an SLP feedback command (approve/retry) as a LessonInteraction row
        /// with InteractionType "SLPFeedback" and IsAcknowledged = false.
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <param name="dto">Feedback command from SLP</param>
        /// <returns>ID of the created LessonInteraction record</returns>
        Task<Guid> RecordSLPFeedbackAsync(Guid sessionId, RecordSLPFeedbackDto dto);

        /// <summary>
        /// Returns the most recent unacknowledged SLP feedback for a session, or null.
        /// Used by robot polling to discover pending commands.
        /// </summary>
        /// <param name="sessionId">ID of the active robot session</param>
        /// <returns>PendingFeedbackResponseDto or null if no pending feedback</returns>
        Task<PendingFeedbackResponseDto?> GetPendingFeedbackAsync(Guid sessionId);

        /// <summary>
        /// Marks a SLPFeedback interaction as acknowledged so the robot does not reprocess it.
        /// </summary>
        /// <param name="sessionId">Owning session (for validation)</param>
        /// <param name="feedbackId">ID of the LessonInteraction to acknowledge</param>
        Task AcknowledgeFeedbackAsync(Guid sessionId, Guid feedbackId);

        /// <summary>
        /// Starts a new lesson in the specified session
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="dto">Lesson start configuration</param>
        /// <returns>The created lesson session</returns>
        Task<RobotSession> StartLessonAsync(Guid sessionId, StartLessonDto dto);

        /// <summary>
        /// Sets the userId on a session (e.g., when a user joins via pairing code)
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <param name="userId">ID of the user joining the session</param>
        Task SetSessionUserIdAsync(Guid sessionId, string userId);

        /// <summary>
        /// Gets tracker events for a session (for analysis)
        /// </summary>
        /// <param name="sessionId">ID of the session</param>
        /// <returns>Collection of tracker events ordered by timestamp descending</returns>
        Task<List<TrackerEventDto>> GetTrackerEventsAsync(Guid sessionId);

        Task<List<LessonInteractionResponseDto>> GetLessonInteractionsAsync(Guid sessionId);
    }
}