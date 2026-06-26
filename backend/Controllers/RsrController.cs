// bloom
// RsrController.cs
// Public proxy controller for the AutoRSR microservice.
// No authentication required — each run is identified by a generated PID.

using bloom.Services;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class RsrController : BloomControllerBase
    {
        private readonly IRsrService _rsr;
        private readonly ILogger<RsrController> _logger;

        public RsrController(IRsrService rsr, ILogger<RsrController> logger)
        {
            _rsr = rsr;
            _logger = logger;
        }

        /// <summary>
        /// Submits a session audio recording to AutoRSR for analysis.
        /// Returns a generated PID that identifies this assessment run.
        /// Expects multipart/form-data: session_audio, age (int), percentile (int), markers (JSON).
        /// </summary>
        [HttpPost("analyze")]
        [RequestSizeLimit(200_000_000)]
        public async Task<IActionResult> Analyze(
            [FromForm] IFormFile sessionAudio,
            [FromForm] int age,
            [FromForm] int percentile,
            [FromForm] string markers)
        {
            try
            {
                var result = await _rsr.AnalyzeAsync(sessionAudio, age, percentile, markers);
                return Ok(result);
            }
            catch (HttpRequestException ex)
            {
                _logger.LogError(ex, "AutoRSR microservice request failed");
                return StatusCode(502, new { message = "AutoRSR service unavailable. Ensure it is running." });
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "RSR analysis error");
                return BadRequest(new { message = ex.Message });
            }
        }

        /// <summary>
        /// Returns a list of past RSR assessments. Optionally filter by PID.
        /// </summary>
        [HttpGet("assessments")]
        public async Task<IActionResult> GetAssessments([FromQuery] string? pid = null)
        {
            try
            {
                var list = await _rsr.GetAssessmentsAsync(pid);
                return Ok(list);
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = ex.Message });
            }
        }

        /// <summary>
        /// Returns the full result for an assessment by its generated PID (e.g. RSR-A3X9K2).
        /// </summary>
        [HttpGet("assessments/pid/{pid}")]
        public async Task<IActionResult> GetAssessmentByPid(string pid)
        {
            try
            {
                var result = await _rsr.GetAssessmentByPidAsync(pid);
                if (result == null) return NotFound();
                return Ok(result);
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = ex.Message });
            }
        }

        /// <summary>
        /// Returns the full result for an assessment by its internal GUID.
        /// </summary>
        [HttpGet("assessments/{id:guid}")]
        public async Task<IActionResult> GetAssessment(Guid id)
        {
            try
            {
                var result = await _rsr.GetAssessmentByIdAsync(id);
                if (result == null) return NotFound();
                return Ok(result);
            }
            catch (Exception ex)
            {
                return BadRequest(new { message = ex.Message });
            }
        }
    }
}
