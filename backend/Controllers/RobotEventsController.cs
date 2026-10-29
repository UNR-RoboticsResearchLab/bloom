// bloom
// RobotEventController.cs
// Robot event logging and retrieval. No session or lesson awareness.

using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages robot device registration and lookup.
    /// Provides CRUD operations and filtering by owner or firmware version.
    /// Has no session or lesson awareness — purely device management.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class RobotEventsController : BloomControllerBase
    {
        
        public RobotEventsController(ILogger<RobotEventsController> logger, IRobotEventService robotEventService)
        {
            _logger = logger;
            _robotEventService = robotEventService;
        }

        [HttpGet("{eventId}")]
        public async Task<IActionResult> GetEventByIdAsync(Guid eventId)
        {
            var robotEvent = await _robotEventService.GetEventByIdAsync(eventId);
            if (robotEvent == null)
            {
                return NotFound();
            }

            return Ok(robotEvent);
        }

        [HttpGet]
        public async Task<IActionResult> GetEventsAsync([FromQuery] RobotEventQueryDto query)
        {
            var events = await _robotEventService.GetEventsAsync(query);
            return Ok(events);
        }
    }

}