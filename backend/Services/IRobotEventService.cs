// bloom
// IRobotEventService.cs
// Interface defining robot event service operations to log robot errors, anomalies,
// timeouts, etc. and retrieve them for analysis and debugging
// Created: 10/23/2025

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public interface IRobotEventService
    {
        /// <summary>
        /// Logs a single event reported by a robot.
        /// </summary>
        /// <returns>The stored RobotEvent</returns>
        /// <exception cref="KeyNotFoundException">Robot (or session, if given) does not exist</exception>
        Task<RobotEvent> LogEventAsync(CreateRobotEventDto dto);

        /// <summary>
        /// Logs several events at once (e.g. a robot flushing events buffered while offline).
        /// </summary>
        /// <returns>The stored RobotEvents</returns>
        Task<ICollection<RobotEvent>> LogEventsAsync(IEnumerable<CreateRobotEventDto> dtos);

        /// <summary>
        /// Gets a specific event by its ID.
        /// </summary>
        /// <returns>RobotEvent or null</returns>
        Task<RobotEvent?> GetEventByIdAsync(Guid eventId);

        /// <summary>
        /// Gets events matching the given filters, newest first.
        /// </summary>
        Task<ICollection<RobotEvent>> GetEventsAsync(RobotEventQueryDto query);

        /// <summary>
        /// Gets all events for a robot, newest first.
        /// </summary>
        Task<ICollection<RobotEvent>> GetEventsByRobotIdAsync(Guid robotId);

        /// <summary>
        /// Gets all events for a session, newest first.
        /// </summary>
        Task<ICollection<RobotEvent>> GetEventsBySessionIdAsync(Guid sessionId);

        /// <summary>
        /// Updates an event's severity or resolution status. Null DTO fields are left unchanged.
        /// </summary>
        /// <returns>The updated RobotEvent, or null if not found</returns>
        Task<RobotEvent?> UpdateEventAsync(Guid eventId, UpdateRobotEventDto dto);

        /// <summary>
        /// Deletes an event.
        /// </summary>
        /// <returns>false if the event was not found</returns>
        Task<bool> DeleteEventAsync(Guid eventId);
    }
}
