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
    /// Retrieves logged robot events by id or query filter.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class RobotEventsController : BloomControllerBase
    {
        private readonly ILogger<RobotEventsController> _logger;
        private readonly IRobotEventService _robotEventService;

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