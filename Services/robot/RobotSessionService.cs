// bloom
// RobotSessionService.cs
// Service layer for managing robot sessions with dual in-memory/database storage
// In-memory storage for current states (real-time access), database for historical snapshots (every 10 changes)
// Created: 11/18/2025

using bloom.Models;
using bloom.Models.dto;
using bloom.Data;
using bloom.Repositories;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class RobotSessionService : IRobotSessionService
    {
        private readonly BloomDbContext _dbContext;
        private readonly IRobotSessionRepository _sessionRepository;
        private readonly IRobotStateRepository _stateRepository;
        private readonly ISessionCodeService _sessionCodeService;

        public RobotSessionService(
            BloomDbContext dbContext,
            IRobotSessionRepository sessionRepository,
            IRobotStateRepository stateRepository,
            ISessionCodeService sessionCodeService)
        {
            _dbContext = dbContext;
            _sessionRepository = sessionRepository;
            _stateRepository = stateRepository;
            _sessionCodeService = sessionCodeService;
        }

        public async Task<RobotSession> StartSessionAsync(Guid robotId, string? userId = null, bool anon = false)
        {

            var session = new RobotSession
            {
                UserId = anon ? null : userId,
                CreatedAt = DateTime.UtcNow,
                LastUpdatedAt = DateTime.UtcNow,
                SessionCode = await GenerateSessionCodeAsync()
            };

            await _sessionRepository.AddAsync(session);

            await AddRobotToSessionAsync(session.Id, robotId);

            var newSession = await _sessionRepository.GetAsync(session.Id);
            if (newSession == null)
                throw new InvalidOperationException($"Failed to retrieve newly created session with ID {session.Id}");
            
            return newSession;
        }

        public async Task EndSessionAsync(Guid sessionId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            // Archive any remaining active states before clearing
            var activeStates = _stateRepository.GetAll(sessionId.ToString());
            foreach (var state in activeStates)
            {
                await _sessionRepository.AddStateHistoryAsync(sessionId, state);
            }

            // Clear in-memory states
            _stateRepository.ClearSession(sessionId.ToString());

            // Update session metadata
            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task AddRobotToSessionAsync(Guid sessionId, Guid robotId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            // Add to in-memory storage
            _stateRepository.Add(sessionId.ToString(), new RobotState
            {
                RobotId = robotId,
                CurrentTask = "pairing",
                Status = "pairing"
            });

            // Increment robot count
            session.Robots += 1;
            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task UpdateRobotStateAsync(Guid sessionId, Guid robotId, RobotState newState)
        {
            if (newState == null)
                throw new ArgumentNullException(nameof(newState));

            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var sessionIdStr = sessionId.ToString();

            // Update in-memory state
            _stateRepository.Add(sessionIdStr, newState);

            // Check if we should archive (every 10th change)
            int changeCount = _stateRepository.GetStateChangeCount(sessionIdStr, robotId);
            if (changeCount % 10 == 0)
            {
                // Archive current state to database
                await _sessionRepository.AddStateHistoryAsync(sessionId, newState);
                _stateRepository.ResetStateChangeCount(sessionIdStr, robotId);
            }

            // Update session timestamp
            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task RemoveRobotFromSessionAsync(Guid sessionId, Guid robotId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var sessionIdStr = sessionId.ToString();
            var lastState = _stateRepository.Get(sessionIdStr, robotId);

            // Archive the last state before removing
            if (lastState != null)
            {
                await _sessionRepository.AddStateHistoryAsync(sessionId, lastState);
            }

            // Remove from in-memory storage
            _stateRepository.Remove(sessionIdStr, robotId);

            // Decrement robot count
            if (session.Robots > 0)
                session.Robots -= 1;

            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task<IEnumerable<RobotState>> GetCurrentStatesAsync(Guid sessionId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            return _stateRepository.GetAll(sessionId.ToString());
        }

        public async Task<IEnumerable<Guid>> GetSessionRobotsAsync(Guid sessionId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var states = _stateRepository.GetAll(sessionId.ToString());
            return states.Select(s => s.RobotId).Distinct().ToList();
        }

        public async Task<IEnumerable<RobotStateHistory>> GetSessionHistoryAsync(Guid sessionId)
        {
            return await _sessionRepository.GetHistoryAsync(sessionId);
        }

        public async Task<RobotSession?> GetSessionAsync(Guid sessionId)
        {
            return await _sessionRepository.GetAsync(sessionId);
        }

        public async Task<IEnumerable<RobotSession>> GetAllSessionsAsync()
        {
            return await _sessionRepository.GetAllAsync();
        }

        public async Task<string> GenerateSessionCodeAsync()
        {
            
            string code;
            int attempts = 0;
            const int maxAttempts = 10;

            // Generate unique code with retry logic
            do
            {
                code = _sessionCodeService.GenerateSessionCode();
                if (!await _sessionCodeService.CodeExistsAsync(code))
                    break;
                attempts++;
            } while (attempts < maxAttempts);

            if (attempts >= maxAttempts)
                throw new InvalidOperationException("Failed to generate unique session code after maximum attempts");

            return code;
        }

        public async Task<RobotSession?> GetSessionByCodeAsync(string code)
        {
            return await _dbContext.RobotSessions.FirstOrDefaultAsync(rs => rs.SessionCode == code);
        }

        public async Task<Guid> LogLessonInteractionAsync(Guid sessionId, LogLessonInteractionDto dto)
        {
            var session = await _sessionRepository.GetAsync(sessionId)
                ?? throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var interaction = new LessonInteraction
            {
                RobotSessionId = sessionId,
                LessonId = session.ActiveLessonId,
                StepId = dto.StepId,
                InteractionType = dto.InteractionType,
                StudentResponse = dto.StudentResponse,
                IsCorrect = dto.IsCorrect,
                ResponseTimeMs = dto.ResponseTimeMs,
                Timestamp = DateTime.UtcNow
            };

            _dbContext.LessonInteractions.Add(interaction);
            await _dbContext.SaveChangesAsync();

            return interaction.Id;
        }

        public async Task UpdateLessonProgressAsync(Guid sessionId, UpdateLessonProgressDto dto)
        {
            var session = await _sessionRepository.GetAsync(sessionId)
                ?? throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            if (session.ActiveLessonId == null)
                throw new InvalidOperationException($"Session {sessionId} has no active lesson");

            if (string.IsNullOrEmpty(session.UserId))
                throw new InvalidOperationException($"Session {sessionId} has no associated user");

            var lessonId = session.ActiveLessonId.Value;
            var studentId = session.UserId;

            var progress = await _dbContext.LessonProgresses
                .FirstOrDefaultAsync(p => p.LessonId == lessonId && p.StudentId == studentId);

            if (progress == null)
            {
                progress = new LessonProgress
                {
                    LessonId = lessonId,
                    StudentId = studentId,
                    LessonStep = dto.CurrentStepId,
                    TotalSteps = dto.TotalSteps,
                    ProgressPercentage = dto.TotalSteps > 0 ? (dto.CompletedSteps * 100) / dto.TotalSteps : 0,
                    LastUpdated = DateTime.UtcNow
                };
                _dbContext.LessonProgresses.Add(progress);
            }
            else
            {
                progress.LessonStep = dto.CurrentStepId;
                progress.TotalSteps = dto.TotalSteps;
                progress.ProgressPercentage = dto.TotalSteps > 0 ? (dto.CompletedSteps * 100) / dto.TotalSteps : 0;
                progress.LastUpdated = DateTime.UtcNow;
                _dbContext.LessonProgresses.Update(progress);
            }

            await _dbContext.SaveChangesAsync();
        }

        public async Task<dynamic?> GetPendingLessonAsync(Guid sessionId)
        {
            try
            {
                // Get the session with its active lesson
                var session = await _dbContext.RobotSessions
                    .Include(s => s.ActiveLesson)
                    .FirstOrDefaultAsync(s => s.Id == sessionId);

                if (session == null)
                    throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

                // If no active lesson, return null (no pending lesson)
                if (session.ActiveLessonId == null || session.ActiveLesson == null)
                    return null;

                // Load the lesson JSON file
                string lessonJson;
                try
                {
                    lessonJson = await File.ReadAllTextAsync(session.ActiveLesson.LessonFileUrl);
                }
                catch (FileNotFoundException)
                {
                    throw new InvalidOperationException($"Lesson file not found at {session.ActiveLesson.LessonFileUrl}");
                }

                // Parse the lesson JSON to ensure it's valid
                var parsedLesson = System.Text.Json.JsonSerializer.Deserialize<dynamic>(lessonJson);

                // Return response with hasPendingLesson flag and the lesson data
                return new
                {
                    hasPendingLesson = true,
                    lesson = parsedLesson
                };
            }
            catch (KeyNotFoundException)
            {
                throw;
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"Error retrieving pending lesson for session {sessionId}: {ex.Message}", ex);
            }
        }
    }
}
