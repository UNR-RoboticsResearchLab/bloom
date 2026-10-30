// bloom
// RobotEventService.cs
// Service implementation for logging and retrieving robot events (errors, anomalies, timeouts, etc.)
// Created: 10/23/2025

using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services.Robots
{
    public class RobotEventService : IRobotEventService
    {
        private const int MaxPageSize = 500;

        private readonly BloomDbContext _dbContext;
        private readonly ILogger<RobotEventService> _logger;

        public RobotEventService(BloomDbContext context, ILogger<RobotEventService> logger)
        {
            _dbContext = context;
            _logger = logger;
        }

        public async Task<RobotEvent> LogEventAsync(CreateRobotEventDto dto)
        {
            var robotEvent = await BuildEventAsync(dto);
            _dbContext.RobotEvents.Add(robotEvent);
            await _dbContext.SaveChangesAsync();

            LogToConsole(robotEvent);
            return robotEvent;
        }

        public async Task<ICollection<RobotEvent>> LogEventsAsync(IEnumerable<CreateRobotEventDto> dtos)
        {
            var events = new List<RobotEvent>();
            foreach (var dto in dtos)
            {
                events.Add(await BuildEventAsync(dto));
            }

            _dbContext.RobotEvents.AddRange(events);
            await _dbContext.SaveChangesAsync();

            foreach (var e in events)
            {
                LogToConsole(e);
            }
            return events;
        }

        public async Task<RobotEvent?> GetEventByIdAsync(Guid eventId)
        {
            return await _dbContext.RobotEvents.FindAsync(eventId);
        }

        public async Task<ICollection<RobotEvent>> GetEventsAsync(RobotEventQueryDto query)
        {
            var events = _dbContext.RobotEvents.AsNoTracking().AsQueryable();

            if (query.RobotId.HasValue)
                events = events.Where(e => e.RobotId == query.RobotId.Value);
            if (query.RobotSessionId.HasValue)
                events = events.Where(e => e.RobotSessionId == query.RobotSessionId.Value);
            if (query.EventType.HasValue)
                events = events.Where(e => e.EventType == query.EventType.Value);
            if (query.MinSeverity.HasValue)
                events = events.Where(e => e.Severity >= query.MinSeverity.Value);
            if (query.IsResolved.HasValue)
                events = events.Where(e => e.IsResolved == query.IsResolved.Value);
            if (query.From.HasValue)
                events = events.Where(e => e.OccurredAt >= query.From.Value);
            if (query.To.HasValue)
                events = events.Where(e => e.OccurredAt <= query.To.Value);

            return await events
                .OrderByDescending(e => e.OccurredAt)
                .Skip(Math.Max(query.Skip, 0))
                .Take(Math.Clamp(query.Take, 1, MaxPageSize))
                .ToListAsync();
        }

        public Task<ICollection<RobotEvent>> GetEventsByRobotIdAsync(Guid robotId) =>
            GetEventsAsync(new RobotEventQueryDto { RobotId = robotId, Take = MaxPageSize });

        public Task<ICollection<RobotEvent>> GetEventsBySessionIdAsync(Guid sessionId) =>
            GetEventsAsync(new RobotEventQueryDto { RobotSessionId = sessionId, Take = MaxPageSize });

        public async Task<RobotEvent?> UpdateEventAsync(Guid eventId, UpdateRobotEventDto dto)
        {
            var robotEvent = await _dbContext.RobotEvents.FindAsync(eventId);
            if (robotEvent == null)
            {
                return null;
            }

            if (dto.Severity.HasValue)
                robotEvent.Severity = dto.Severity.Value;

            if (dto.ResolutionNotes != null)
                robotEvent.ResolutionNotes = dto.ResolutionNotes;

            if (dto.IsResolved.HasValue && dto.IsResolved.Value != robotEvent.IsResolved)
            {
                robotEvent.IsResolved = dto.IsResolved.Value;
                robotEvent.ResolvedAt = dto.IsResolved.Value ? DateTime.UtcNow : null;
            }

            await _dbContext.SaveChangesAsync();
            return robotEvent;
        }

        public async Task<bool> DeleteEventAsync(Guid eventId)
        {
            var robotEvent = await _dbContext.RobotEvents.FindAsync(eventId);
            if (robotEvent == null)
            {
                return false;
            }

            _dbContext.RobotEvents.Remove(robotEvent);
            await _dbContext.SaveChangesAsync();
            return true;
        }

        private async Task<RobotEvent> BuildEventAsync(CreateRobotEventDto dto)
        {
            if (string.IsNullOrWhiteSpace(dto.Message))
            {
                throw new ArgumentException("Event message is required");
            }

            if (!await _dbContext.Robots.AnyAsync(r => r.Id == dto.RobotId))
            {
                throw new KeyNotFoundException($"Robot {dto.RobotId} not found");
            }

            if (dto.RobotSessionId.HasValue &&
                !await _dbContext.RobotSessions.AnyAsync(s => s.Id == dto.RobotSessionId.Value))
            {
                throw new KeyNotFoundException($"Session {dto.RobotSessionId} not found");
            }

            var now = DateTime.UtcNow;
            return new RobotEvent
            {
                RobotId = dto.RobotId,
                RobotSessionId = dto.RobotSessionId,
                EventType = dto.EventType,
                Severity = dto.Severity,
                Source = dto.Source,
                Code = dto.Code,
                Message = dto.Message,
                Details = dto.Details,
                // Don't trust robot clocks into the future
                OccurredAt = dto.OccurredAt.HasValue && dto.OccurredAt.Value <= now
                    ? dto.OccurredAt.Value.ToUniversalTime()
                    : now,
                ReceivedAt = now
            };
        }

        private void LogToConsole(RobotEvent e)
        {
            var level = e.Severity >= RobotEventSeverity.High ? LogLevel.Error : LogLevel.Warning;
            _logger.Log(level, "Robot {RobotId} {EventType} ({Severity}) from {Source}: {Message}",
                e.RobotId, e.EventType, e.Severity, e.Source, e.Message);
        }
    }
}
