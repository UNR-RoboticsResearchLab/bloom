// bloom
// SLPClientController.cs
// API controller for managing SLPClient records (student-SLP associations)

using System.Security.Claims;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Manages SLPClient records, which associate a student with one or more SLP teachers.
    /// SLPs create client profiles, view their clients, and retrieve progress for each student.
    /// </summary>
    [Authorize]
    [ApiController]
    [Route("api/[controller]")]
    public class SLPClientController : ControllerBase
    {
        private readonly ISLPClientService _slpClientService;
        private readonly ILogger<SLPClientController> _logger;

        public SLPClientController(ISLPClientService slpClientService, ILogger<SLPClientController> logger)
        {
            _slpClientService = slpClientService;
            _logger = logger;
        }

        /// <summary>
        /// Create a new SLPClient profile, linking a student to the requesting SLP.
        /// The requesting SLP is automatically added as a teacher on the client.
        /// </summary>
        [HttpPost]
        public async Task<IActionResult> CreateClient([FromBody] CreateSLPClientDto dto)
        {
            if (!ModelState.IsValid)
                return BadRequest(ModelState);

            var slpId = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var result = await _slpClientService.CreateClientAsync(dto.Name, slpId, dto.StudentId);
            if (result == null)
                return NotFound(new { message = $"Student {dto.StudentId} not found." });

            _logger.LogInformation("SLPClient {ClientId} created for student {StudentId} by SLP {SlpId}", result.Id, dto.StudentId, slpId);

            return Ok(result);
        }

        /// <summary>
        /// List all SLPClients for the authenticated slp
        /// </summary>
        [HttpGet]
        public async Task<IActionResult> GetMyClients()
        {
            var slpId = User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (string.IsNullOrEmpty(slpId))
                return Unauthorized();

            var clients = await _slpClientService.GetClientsForSlpAsync(slpId);
            return Ok(clients);
        }

        /// <summary>
        /// Get full details for an SLPClient, including the student's assignments and lesson progress.
        /// </summary>
        [HttpGet("{id}")]
        public async Task<IActionResult> GetClient(Guid id)
        {
            var client = await _slpClientService.GetClientAsync(id);
            if (client == null)
                return NotFound(new { message = $"SLPClient {id} not found." });

            return Ok(client);
        }

        /// <summary>
        /// Add another SLP as a teacher on an existing client.
        /// </summary>
        [HttpPost("{id}/teachers/{teacherId}")]
        public async Task<IActionResult> AddTeacher(Guid id, string teacherId)
        {
            var result = await _slpClientService.AddTeacherAsync(id, teacherId);
            return result switch
            {
                AddTeacherResult.Added => Ok(new { message = "Teacher added.", ClientId = id, TeacherId = teacherId }),
                AddTeacherResult.ClientNotFound => NotFound(new { message = $"SLPClient {id} not found." }),
                AddTeacherResult.TeacherNotFound => NotFound(new { message = $"Account {teacherId} not found." }),
                AddTeacherResult.AlreadyAssociated => Conflict(new { message = "Teacher already associated with this client." }),
                _ => StatusCode(500)
            };
        }
    }
}
