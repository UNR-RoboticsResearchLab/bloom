// bloom
// RobotController.cs
// Robot device registration and lookup. No session or lesson awareness.

using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using bloom.Services.RemoteShell;
using Microsoft.Extensions.Options;
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
        private readonly IShellRelay _shellRelay;
        private readonly IShellAuditService _shellAudit;
        private readonly IOptions<RemoteShellOptions> _shellOptions;

        public RobotController(
            ILogger<RobotController> logger,
            IRobotService robotService,
            IShellRelay shellRelay,
            IShellAuditService shellAudit,
            IOptions<RemoteShellOptions> shellOptions)
        {
            _logger = logger;
            _robotService = robotService;
            _shellRelay = shellRelay;
            _shellAudit = shellAudit;
            _shellOptions = shellOptions;
        }

        /// <summary>
        /// Provisions a new robot device. Returns the created record plus its login secret --
        /// the only time the secret is ever shown. Put RobotId + Secret in the robot's .env.
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpPost("register")]
        public async Task<ActionResult<string>> RegisterRobot([FromBody] RobotDto robot)
        {
            try
            {
                var result = await _robotService.RegisterRobotAsync(robot);
                if (result != Guid.Empty)
                {
                    var secret = await _robotService.IssueSecretAsync(result);
                    var newRobot = await _robotService.GetRobotByIdAsync(result);
                    return Ok(new { Message = "Robot registered successfully", Robot = newRobot, Secret = secret });
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
        public async Task<ActionResult<string>> LoginRobot([FromBody] RobotLoginDto loginDto)
        {
            try
            {
                var result = await _robotService.LoginRobotAsync(loginDto);
                if (result != null)
                    return Ok(new { Message = "Login successful", Token = result.Token, RobotId = result.RobotId });

                return Unauthorized(new { Message = "Invalid credentials" });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error logging in robot");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Replaces a robot's login secret and invalidates all of its existing tokens. Returns the
        /// new secret once.
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpPost("{id}/credentials")]
        public async Task<ActionResult<string>> RotateCredentials(Guid id)
        {
            var secret = await _robotService.IssueSecretAsync(id);
            if (secret == null)
                return NotFound(new { Message = "Robot not found" });

            _logger.LogWarning("Robot {RobotId} credentials rotated by {UserId}", id, GetCurrentUserId());
            await _shellRelay.DisconnectRobotAsync(id, ShellEndReason.Revoked);
            return Ok(new { RobotId = id, Secret = secret });
        }

        /// <summary>
        /// Rotates a robot's secret by pushing it to the robot's agent over its live tunnel, which
        /// applies it on the robot (agent + bloom_node). The old secret stays valid until the robot
        /// logs in with the new one, so a failed or lost push never locks it out. The secret is
        /// never returned. 409: robot offline or a push is already running; 502: the agent refused;
        /// 504: no confirmation (the robot may or may not have it -- both secrets keep working).
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpPost("{id}/credentials/push")]
        public async Task<ActionResult<string>> PushCredentials(Guid id)
        {
            if (await _robotService.GetRobotByIdAsync(id) == null)
                return NotFound(new { Message = "Robot not found" });
            if (!_shellRelay.IsOnline(id))
                return Conflict(new { Message = "The robot is offline. Download a provisioning file instead." });

            var secret = await _robotService.BeginSecretRotationAsync(id);
            if (secret == null)
                return Conflict(new { Message = "The robot has no credentials to rotate. Issue credentials first." });

            var result = await _shellRelay.PushSecretAsync(id, secret,
                TimeSpan.FromSeconds(_shellOptions.Value.SecretPushTimeoutSeconds));

            switch (result.Status)
            {
                case SecretPushStatus.Applied:
                    _logger.LogWarning("Robot {RobotId} credentials pushed by {UserId}", id, GetCurrentUserId());
                    // Reconnect now so the agent logs in with (and promotes) the new secret.
                    await _shellRelay.DisconnectRobotAsync(id, ShellEndReason.CredentialsRotated);
                    return Ok(new { Message = "Credentials delivered; the robot is reconnecting" });

                case SecretPushStatus.TimedOut:
                case SecretPushStatus.ConnectionLost:
                    // Unknown whether the robot applied it, so keep the pending secret: whichever
                    // one the robot ends up holding will work.
                    _logger.LogWarning("Credential push to robot {RobotId} unconfirmed ({Status})", id, result.Status);
                    return StatusCode(StatusCodes.Status504GatewayTimeout, new
                    {
                        Message = "The robot didn't confirm. It may or may not have the new secret; both the old and new secret keep working, so check the fleet view."
                    });

                case SecretPushStatus.Rejected:
                    await _robotService.ClearPendingSecretAsync(id);
                    _logger.LogWarning("Robot {RobotId} agent refused a credential push: {Message}", id, result.Message);
                    return StatusCode(StatusCodes.Status502BadGateway, new
                    {
                        Message = $"The robot couldn't apply it: {result.Message ?? "unknown error"}. Nothing changed."
                    });

                default:  // Offline (dropped just now) or Busy
                    await _robotService.ClearPendingSecretAsync(id);
                    return Conflict(new
                    {
                        Message = result.Status == SecretPushStatus.Busy
                            ? "A credential push to this robot is already in progress."
                            : "The robot went offline. Download a provisioning file instead."
                    });
            }
        }

        /// <summary>
        /// Revokes a robot: removes its secret and invalidates all of its tokens. It can't log in
        /// again until an admin rotates new credentials.
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpPost("{id}/revoke")]
        public async Task<ActionResult<string>> RevokeRobot(Guid id)
        {
            if (!await _robotService.RevokeCredentialsAsync(id))
                return NotFound(new { Message = "Robot not found" });

            _logger.LogWarning("Robot {RobotId} revoked by {UserId}", id, GetCurrentUserId());
            await _shellRelay.DisconnectRobotAsync(id, ShellEndReason.Revoked);
            return Ok(new { Message = "Robot revoked" });
        }

        /// <summary>
        /// Every robot with its agent status (online, reported version) and current shell holder.
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpGet("fleet")]
        public async Task<ActionResult<IEnumerable<RobotFleetDto>>> GetFleet()
        {
            var robots = await _robotService.GetAllRobotsAsync();
            return Ok(robots.OrderBy(r => r.Name).Select(r =>
            {
                var shell = _shellRelay.GetActiveSession(r.Id);
                return new RobotFleetDto
                {
                    Id = r.Id,
                    Name = r.Name,
                    Model = r.Model,
                    FirmwareVersion = r.FirmwareVersion,
                    Online = _shellRelay.IsOnline(r.Id),
                    Provisioned = r.SecretHash != null,
                    AgentVersion = r.AgentVersion,
                    AgentCommit = r.AgentCommit,
                    LastSeenAt = r.LastSeenAt,
                    ShellHeldBy = shell?.AdminEmail ?? shell?.AdminUserId,
                    ShellSince = shell?.StartedAt,
                };
            }));
        }

        /// <summary>
        /// Remote shell audit log for one robot, newest first: who, when, and why it ended.
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpGet("{id}/shell-sessions")]
        public async Task<ActionResult<IEnumerable<ShellSessionAudit>>> GetShellSessions(Guid id)
        {
            return Ok(await _shellAudit.GetForRobotAsync(id));
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
        /// Permanently removes a robot from the registry (Admin only). Its shell audit log is kept.
        /// </summary>
        [Authorize(Policy = "AdminOnly")]
        [HttpDelete("{id}")]
        public async Task<ActionResult<string>> DeleteRobot(Guid id)
        {
            try
            {
                // Close its tunnel and shell first so a deleted robot can't keep a session open.
                await _shellRelay.DisconnectRobotAsync(id, ShellEndReason.RobotDeleted);
                var result = await _robotService.DeleteRobotAsync(id);
                if (result)
                {
                    _logger.LogWarning("Robot {RobotId} deleted by {UserId}", id, GetCurrentUserId());
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

        /// <summary>
        /// Returns all distinct behavior names found across lesson steps.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
        [HttpGet("behaviors")]
        public async Task<ActionResult<ICollection<Behavior>>> GetAvailableBehaviors()
        {
            try
            {
                var behaviors = await _robotService.GetAvailableBehaviorsAsync();
                return Ok(behaviors);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving available behaviors");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns all distinct motor sequence names found across lesson steps.
        /// </summary>
        [Authorize(Policy = "JwtOrCookie")]
        [HttpGet("motorsequences")]
        public async Task<ActionResult<ICollection<string>>> GetAvailableMotorSequences()
        {
            try
            {
                var sequences = await _robotService.GetAvailableMotorSequencesAsync();
                return Ok(sequences);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error retrieving available motor sequences");
                return BadRequest(new { ex.Message });
            }
        }
    }
}
