// bloom
// RobotController.cs
// Robot device registration and lookup. No session or lesson awareness.

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
    public class RobotController : BloomControllerBase
    {
        private readonly ILogger<RobotController> _logger;
        private readonly IRobotService _robotService;

        public RobotController(ILogger<RobotController> logger, IRobotService robotService)
        {
            _logger = logger;
            _robotService = robotService;
        }

        /// <summary>
        /// Registers a new robot device and returns the created record.
        /// </summary>
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

        ///<summary>
        /// Logs in a registered robot device by validating its credentials and returning jwt token.
        /// </summary>
        [HttpPost("login")]
        public async Task<ActionResult<string>> LoginRobot([FromBody] RobotDto loginDto)
        {
            try
            {
                var token = await _robotService.LoginRobotAsync(loginDto);
                if (token != null)
                    return Ok(new { Message = "Login successful", Token = token });

                return Unauthorized(new { Message = "Invalid credentials" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error logging in robot");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Updates the configuration or metadata of an existing robot.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
        [HttpPut("{id}")]
        public async Task<ActionResult<string>> UpdateRobot(Guid id, [FromBody] RobotDto robot)
        {
            try
            {
                var result = await _robotService.UpdateRobotAsync(id, robot);
                if (result)
                    return Ok(new { Message = "Robot updated successfully" });

                return NotFound(new { Message = "Robot not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error updating robot");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Permanently removes a robot from the registry.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
        [HttpDelete("{id}")]
        public async Task<ActionResult<string>> DeleteRobot(Guid id)
        {
            try
            {
                var result = await _robotService.DeleteRobotAsync(id);
                if (result)
                    return Ok(new { Message = "Robot deleted successfully" });

                return NotFound(new { Message = "Robot not found" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error deleting robot");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns a single robot by ID.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
        [HttpGet("{id}")]
        public async Task<ActionResult<Robot>> GetRobot(Guid id)
        {
            try
            {
                var robot = await _robotService.GetRobotByIdAsync(id);
                if (robot == null)
                    return NotFound(new { Message = "Robot not found" });

                return Ok(robot);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving robot");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns all registered robots. Intended for admin use.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
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

        /// <summary>
        /// Returns all robots registered to the specified user.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
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

        /// <summary>
        /// Returns all robots running the specified firmware version. Useful for fleet management.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
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
