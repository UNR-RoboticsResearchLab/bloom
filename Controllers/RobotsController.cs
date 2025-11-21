using Microsoft.AspNetCore.Mvc;
using System.Collections.Generic;
using bloom.Models;
using bloom.Services;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class RobotsController : ControllerBase
    {
        private readonly ILogger<RobotsController> _logger;
        private readonly IRobotService _robotService;

        
        public RobotsController(ILogger<RobotsController> logger, IRobotService robotService)
        {
            _logger = logger;
            _robotService = robotService;
        }

        [HttpPost("register")]
        public ActionResult<string> RegisterRobot([FromBody] RobotDto robot)
        {
            try
            {
                var result = _robotService.RegisterRobotAsync(robot);
                if (result)
                {
                    return Ok("Robot registered successfully");
                }
                return BadRequest("Failed to register robot");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error registering robot");
                return BadRequest(ex.Message);
            }
        }

        [HttpPut("{id}")]
        public ActionResult<string> UpdateRobot(Guid id, [FromBody] RobotDto robot)
        {
            try
            {
                var result = _robotService.UpdateRobotAsync(id, robot);
                if (result)
                {
                    return Ok("Robot updated successfully");
                }
                return NotFound("Robot not found");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error updating robot");
                return BadRequest(ex.Message);
            }
        }

        [HttpDelete("{id}")]
        public ActionResult<string> DeleteRobot(Guid id)
        {
            try
            {
                var result = _robotService.DeleteRobotAsync(id);
                if (result)
                {
                    return Ok("Robot deleted successfully");
                }
                return NotFound("Robot not found");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error deleting robot");
                return BadRequest(ex.Message);
            }
        }

        [HttpGet("{id}")]
        public ActionResult<Robot> GetRobot(Guid id)
        {
            try
            {
                var robot = _robotService.GetRobotByIdAsync(id);
                if (robot == null)
                {
                    return NotFound("Robot not found");
                }
                return Ok(robot);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robot");
                return BadRequest(ex.Message);
            }
        }

        [HttpGet]
        public ActionResult<ICollection<Robot>> GetAllRobots()
        {
            try
            {
                var robots = _robotService.GetAllRobotsAsync();
                return Ok(robots);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots");
                return BadRequest(ex.Message);
            }
        }

        [HttpGet("user/{userId}")]
        public ActionResult<ICollection<Robot>> GetRobotsByUserId(string userId)
        {
            try
            {
                var robots = _robotService.GetRobotsByUserIdAsync(userId);
                return Ok(robots);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots for user");
                return BadRequest(ex.Message);
            }
        }

        [HttpGet("firmware/{firmwareVersion}")]
        public ActionResult<ICollection<Robot>> GetRobotsByFirmwareVersion(string firmwareVersion)
        {
            try
            {
                var robots = _robotService.GetRobotsByFirmwareVersion(firmwareVersion);
                return Ok(robots);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robots by firmware version");
                return BadRequest(ex.Message);
            }
        }

    }
}