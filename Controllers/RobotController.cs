using bloom.Models;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers;

[Route("[controller]")]
class RobotController : ControllerBase
{
    [HttpGet]
    [Route("health")]
    public IActionResult HealthCheck()
    {
        return Ok(new { Status = "Healthy", Timestamp = DateTime.UtcNow });
    }

    [HttpGet]
    [Route("start-session")]
    public IActionResult StartSession()
    {
        // Placeholder for starting a robot session
        return Ok(new { Message = "Robot session started." });
    }

    [HttpGet]
    [Route("stop-session")]
    public IActionResult StopSession()
    {
        if (!ModelState.IsValid)
        {
            return BadRequest(ModelState);
        }

        // Placeholder for stopping a robot session
        return Ok(new { Message = "Robot session stopped." });

    }

    [HttpPost]
    [Route("register-robot")]
    public IActionResult RegisterRobot([FromBody] RegisterRobotDto robot)
    {
        if (!ModelState.IsValid)
        {
            return BadRequest(ModelState);
        }


        // Placeholder for registering a robot
        return Ok(new { Message = $"Robot {robot.Name} registered successfully." });
    }

    [HttpGet]
    [Route("session-status")]
    public IActionResult SessionStatus()
    {
        // returns the status of the current robot session
        return Ok(new { Status = "Active", Timestamp = DateTime.UtcNow });
    }
    
    
}