using Microsoft.AspNetCore.Mvc;
using System.Collections.Generic;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages robot device registration and lookup.
    /// Provides CRUD operations for robot records and supports
    /// filtering by owner user ID or firmware version.
    /// Has no session or lesson awareness — purely device management.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class RobotController : ControllerBase
    {
        private readonly ILogger<RobotController> _logger;
        private readonly IRobotService _robotService;

        
        public RobotController(ILogger<RobotController> logger, IRobotService robotService)
        {
            _logger = logger;
            _robotService = robotService;
        }

        [HttpPost("register")]
        public async Task<ActionResult<string>> RegisterRobot([FromBody] RobotDto robot)
        {
            try
            {
                var result = await _robotService.RegisterRobotAsync(robot);

                if (result != Guid.Empty)
                {
                    var newRobot = await _robotService.GetRobotByIdAsync(result);
                    return Ok(new { Message = "Robot registered successfully", Robot = newRobot });
                }
                return BadRequest(new { Message = "Failed to register robot" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error registering robot");
                return BadRequest(new { ex.Message });
            }
        }

        [HttpPut("{id}")]
        public async Task<ActionResult<string>> UpdateRobot(Guid id, [FromBody] RobotDto robot)
        {
            try
            {
                var result = await _robotService.UpdateRobotAsync(id, robot);
                if (result)
                {
                    return Ok(new { Message = "Robot updated successfully" });
                }
                return NotFound(new { Message = "Robot not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error updating robot");
                return BadRequest(new { ex.Message });
            }
        }

        [HttpDelete("{id}")]
        public async Task<ActionResult<string>> DeleteRobot(Guid id)
        {
            try
            {
                var result = await _robotService.DeleteRobotAsync(id);
                if (result)
                {
                    return Ok(new { Message = "Robot deleted successfully" });
                }
                return NotFound(new { Message = "Robot not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error deleting robot");
                return BadRequest(new { ex.Message });
            }
        }

        [HttpGet("{id}")]
        public async Task<ActionResult<Robot>> GetRobot(Guid id)
        {
            try
            {
                var robot = await _robotService.GetRobotByIdAsync(id);
                if (robot == null)
                {
                    return NotFound(new { Message = "Robot not found" });
                }
                return Ok(robot);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robot");
                return BadRequest(new { ex.Message });
            }
        }

        [HttpGet]
        public async Task<ActionResult<ICollection<Robot>>> GetAllRobots()
        {
            try
            {
                var robots = await _robotService.GetAllRobotsAsync();
                return Ok(robots);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots");
                return BadRequest(new { ex.Message });
            }
        }

        [HttpGet("user/{userId}")]
        public async Task<ActionResult<ICollection<Robot>>> GetRobotsByUserId(string userId)
        {
            try
            {
                var robots = await _robotService.GetRobotsByUserIdAsync(userId);
                return Ok(robots);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots for user");
                return BadRequest(new { ex.Message });
            }
        }

        [HttpGet("firmware/{firmwareVersion}")]
        public async Task<ActionResult<ICollection<Robot>>> GetRobotsByFirmwareVersion(string firmwareVersion)
        {
            try
            {
                var robots = await _robotService.GetRobotsByFirmwareVersion(firmwareVersion);
                return Ok(robots);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots by firmware version");
                return BadRequest(new { ex.Message });
            }
        }

    }
}