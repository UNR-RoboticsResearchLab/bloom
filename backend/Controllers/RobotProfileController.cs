// bloom
// RobotProfileController.cs
// Lets a student customize their own robot (nickname, personality, etc.), decoupled
// from physical Robot device registration/auth.

using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages per-account robot customization profiles.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    public class RobotProfileController : BloomControllerBase
    {
        private readonly ILogger<RobotProfileController> _logger;
        private readonly IRobotProfileService _robotProfileService;

        public RobotProfileController(ILogger<RobotProfileController> logger, IRobotProfileService robotProfileService)
        {
            _logger = logger;
            _robotProfileService = robotProfileService;
        }

        /// <summary>
        /// Returns the currently authenticated user's robot profile.
        /// </summary>
        [Authorize]
        [HttpGet("me")]
        public async Task<IActionResult> GetMyProfile()
        {
            var userId = GetCurrentUserId();
            if (string.IsNullOrEmpty(userId))
                return Unauthorized();

            var profile = await _robotProfileService.GetByAccountIdAsync(userId);
            if (profile == null)
                return NotFound(new { Message = "No robot profile has been set up yet." });

            return Ok(profile);
        }

        /// <summary>
        /// Creates or updates the currently authenticated user's robot profile.
        /// </summary>
        [Authorize]
        [HttpPut("me")]
        public async Task<IActionResult> UpdateMyProfile([FromBody] RobotProfileDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var userId = GetCurrentUserId();
            if (string.IsNullOrEmpty(userId))
                return Unauthorized();

            try
            {
                var profile = await _robotProfileService.UpsertAsync(userId, dto);
                return Ok(new { Message = "Robot profile saved successfully", Profile = profile });
            }
            catch (ArgumentException ex)
            {
                return BadRequest(new { ex.Message });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error saving robot profile");
                return BadRequest(new { ex.Message });
            }
        }

        /// <summary>
        /// Returns the robot profile for a specific account. Intended for SLP/teacher views.
        /// </summary>
        [Authorize]
        [HttpGet("{accountId}")]
        public async Task<IActionResult> GetProfileByAccountId(string accountId)
        {
            var profile = await _robotProfileService.GetByAccountIdAsync(accountId);
            if (profile == null)
                return NotFound(new { Message = "No robot profile has been set up yet." });

            return Ok(profile);
        }

        /// <summary>
        /// Returns the fixed list of selectable personality presets.
        /// </summary>
        [Authorize]
        [HttpGet("presets")]
        public IActionResult GetPersonalityPresets()
        {
            return Ok(_robotProfileService.GetPersonalityPresets());
        }

        /// <summary>
        /// Returns the fixed list of selectable TTS voice presets.
        /// </summary>
        [Authorize]
        [HttpGet("voice-presets")]
        public IActionResult GetVoicePresets()
        {
            return Ok(_robotProfileService.GetVoicePresets());
        }
    }
}
