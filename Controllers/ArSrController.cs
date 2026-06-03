using System.Security.Claims;
using System.Text.Json;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using bloom.Services;
using Microsoft.AspNetCore.Authentication;
using Microsoft.AspNetCore.Authentication.Cookies;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class ArSrController : ControllerBase
    {
        private readonly IArSrService _arSrService;
        private readonly ArSrTranscriptionService _transcription;
        private readonly UserManager<Account> _userManager;
        private readonly ILogger<ArSrController> _logger;

        private static readonly List<string> GroundTruth = new()
        {
            "The big football player washed the car with the hose.",
            "All of the pictures were colored by his little sister.",
            "The rose bushes were planted yesterday by the girl scouts.",
            "The happy little girl kicked the ball over the fence.",
            "His little brother cleaned the dirty dishes and cups.",
            "A special cage was made to hold the dangerous animals.",
            "Everybody in my school colored Easter eggs for the picnic.",
            "A new hole was dug for the kid's swimming pool.",
            "Only the first graders made a birdhouse for their parents.",
            "My little sister's dog caught the ball on the first bounce.",
            "The soccer ball was kicked into the school's parking lot.",
            "The lion's teeth were cleaned with a giant toothbrush.",
            "Some of the kids dug holes in the sand two feet deep.",
            "The little white mouse was caught by our neighbor's cat.",
            "The second grade students planted coconuts in the garden.",
            "The dirty clothes were washed with soap one more time.",
        };

        public ArSrController(
            IArSrService arSrService,
            ArSrTranscriptionService transcription,
            UserManager<Account> userManager,
            ILogger<ArSrController> logger)
        {
            _arSrService   = arSrService;
            _transcription = transcription;
            _userManager   = userManager;
            _logger        = logger;
        }

        // ── Sentences ────────────────────────────────────────────────────────

        [HttpGet("sentences")]
        public IActionResult GetSentences() =>
            Ok(GroundTruth.Select((text, i) => new { id = i + 1, text }));

        // ── Participant auth ─────────────────────────────────────────────────

        /// <summary>Create a new participant account (no password required).</summary>
        [HttpPost("participants")]
        [Authorize]
        public async Task<IActionResult> CreateParticipant([FromBody] ArSrEnrollRequest request)
        {
            var existing = _userManager.Users.FirstOrDefault(u => u.ParticipantCode == request.ParticipantCode);
            if (existing != null)
                return Conflict(new { message = "Participant code already exists." });

            var account = new Account
            {
                UserName        = request.ParticipantCode,
                ParticipantCode = request.ParticipantCode,
                Role            = "Participant",
                CreatedDate     = DateTime.UtcNow,
            };

            var result = await _userManager.CreateAsync(account);
            if (!result.Succeeded)
                return BadRequest(new { message = "Failed to create participant.", errors = result.Errors });

            await _userManager.AddToRoleAsync(account, "Participant");

            var enrollment = await _arSrService.EnrollParticipantAsync(request, account.Id);
            return Ok(MapEnrollment(enrollment, account.ParticipantCode!));
        }

        /// <summary>Log in with participant code only — issues a session cookie.</summary>
        [HttpPost("participants/login")]
        public async Task<IActionResult> ParticipantLogin([FromBody] ArSrParticipantLoginRequest request)
        {
            var account = _userManager.Users.FirstOrDefault(u => u.ParticipantCode == request.ParticipantCode);
            if (account == null)
                return Unauthorized(new { message = "Participant code not found." });

            var claims = new List<Claim>
            {
                new(ClaimTypes.NameIdentifier, account.Id),
                new(ClaimTypes.Role, "Participant"),
            };
            var identity  = new ClaimsIdentity(claims, CookieAuthenticationDefaults.AuthenticationScheme);
            var principal = new ClaimsPrincipal(identity);
            await HttpContext.SignInAsync(CookieAuthenticationDefaults.AuthenticationScheme, principal);

            return Ok(new { message = "Logged in", participantCode = account.ParticipantCode });
        }

        // ── Enrollments ──────────────────────────────────────────────────────

        [HttpGet("enrollments")]
        [Authorize]
        public async Task<IActionResult> GetEnrollments()
        {
            var enrollments = await _arSrService.GetAllEnrollmentsAsync();
            var dtos = new List<ArSrEnrollmentDto>();

            foreach (var e in enrollments)
            {
                var acct = await _userManager.FindByIdAsync(e.ParticipantAccountId);
                dtos.Add(MapEnrollment(e, acct?.ParticipantCode ?? e.ParticipantAccountId));
            }

            return Ok(dtos);
        }

        [HttpGet("enrollments/{enrollmentId}")]
        [Authorize]
        public async Task<IActionResult> GetEnrollment(Guid enrollmentId)
        {
            var enrollment = await _arSrService.GetEnrollmentAsync(enrollmentId);
            if (enrollment == null) return NotFound();

            var acct = await _userManager.FindByIdAsync(enrollment.ParticipantAccountId);
            return Ok(MapEnrollment(enrollment, acct?.ParticipantCode ?? enrollment.ParticipantAccountId));
        }

        // ── Sessions ─────────────────────────────────────────────────────────

        [HttpGet("sessions")]
        [Authorize]
        public async Task<IActionResult> GetSessions()
        {
            var sessions = await _arSrService.GetAllSessionsAsync();
            return Ok(sessions.Select(MapSession));
        }

        [HttpGet("sessions/{sessionId}")]
        [Authorize]
        public async Task<IActionResult> GetSession(Guid sessionId)
        {
            var session = await _arSrService.GetSessionAsync(sessionId);
            if (session == null) return NotFound();
            return Ok(MapSession(session));
        }

        /// <summary>
        /// Submit a recorded RSR session for transcription, scoring, and storage.
        /// Expects multipart/form-data:
        ///   session_audio  — webm/wav recording of the full session
        ///   enrollmentId   — Guid
        ///   ageMonths      — int
        ///   percentile     — int (default 5)
        ///   markers        — JSON array of {sentence_id, start_ms, end_ms}
        ///   videoFilePath  — optional string
        ///   notes          — optional string
        /// </summary>
        [HttpPost("sessions")]
        [Authorize]
        public async Task<IActionResult> SubmitSession()
        {
            if (!Request.HasFormContentType)
                return BadRequest(new { message = "Multipart form required." });

            var form = Request.Form;

            if (!Guid.TryParse(form["enrollmentId"], out var enrollmentId))
                return BadRequest(new { message = "Invalid enrollmentId." });

            if (!int.TryParse(form["ageMonths"], out var ageMonths))
                return BadRequest(new { message = "Invalid ageMonths." });

            int.TryParse(form["percentile"], out var percentile);
            if (percentile == 0) percentile = 5;

            var markersJson = form["markers"].ToString();
            if (string.IsNullOrWhiteSpace(markersJson))
                return BadRequest(new { message = "markers are required." });

            if (!form.Files.Any())
                return BadRequest(new { message = "session_audio file is required." });

            var audioFile = form.Files["session_audio"];
            if (audioFile == null)
                return BadRequest(new { message = "session_audio file is required." });

            try
            {
                using var stream = audioFile.OpenReadStream();
                var analysis = await _transcription.AnalyzeAsync(
                    stream, audioFile.FileName, ageMonths, percentile, markersJson);

                var sessionRequest = new ArSrSessionRequest
                {
                    EnrollmentId       = enrollmentId,
                    AgeMonths          = ageMonths,
                    Percentile         = percentile,
                    VideoFilePath      = form["videoFilePath"].ToString(),
                    AdministratorNotes = form["notes"].ToString(),
                };

                var session = await _arSrService.CreateSessionAsync(sessionRequest, analysis);
                return Ok(MapSession(session));
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error analyzing RSR session");
                return StatusCode(500, new { message = "Transcription service error.", detail = ex.Message });
            }
        }

        // ── Health ───────────────────────────────────────────────────────────

        [HttpGet("health")]
        public async Task<IActionResult> Health()
        {
            var ok = await _transcription.IsHealthyAsync();
            return Ok(new { transcriptionService = ok ? "up" : "down" });
        }

        // ── Mapping helpers ──────────────────────────────────────────────────

        private static ArSrEnrollmentDto MapEnrollment(ArSrEnrollment e, string code) => new()
        {
            Id                    = e.Id,
            ParticipantAccountId  = e.ParticipantAccountId,
            ParticipantCode       = code,
            Condition             = e.Condition,
            EnrolledAt            = e.EnrolledAt,
            Gender                = e.Gender,
            Ethnicity             = e.Ethnicity,
            NativeLanguage        = e.NativeLanguage,
            HearingLoss           = e.HearingLoss,
            ReceivingSpeechTherapy = e.ReceivingSpeechTherapy,
            SessionCount          = e.Sessions?.Count ?? 0,
        };

        private static ArSrSessionDto MapSession(ArSrSession s) => new()
        {
            Id                 = s.Id,
            EnrollmentId       = s.EnrollmentId,
            ParticipantCode    = s.Enrollment?.ParticipantAccountId ?? string.Empty,
            AgeMonths          = s.AgeMonths,
            Percentile         = s.Percentile,
            TotalScore         = s.TotalScore,
            Result             = s.Result,
            VideoFilePath      = s.VideoFilePath,
            CreatedAt          = s.CreatedAt,
            AdministratorNotes = s.AdministratorNotes,
            SentenceResults    = s.SentenceResults.OrderBy(r => r.SentenceNumber).Select(r => new ArSrSentenceResultDto
            {
                SentenceNumber = r.SentenceNumber,
                GroundTruth    = r.GroundTruth,
                Response       = r.Response,
                Errors         = r.Errors,
                Score          = r.Score,
                EditScript     = r.EditScript,
            }).ToList(),
            Recordings = s.Recordings.OrderBy(r => r.SentenceNumber).Select(r => new ArSrRecordingDto
            {
                SentenceNumber = r.SentenceNumber,
                FilePath       = r.FilePath,
            }).ToList(),
        };
    }
}
