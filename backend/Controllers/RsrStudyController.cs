using System.Security.Claims;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers;

/// <summary>
/// Private, append-only study archives. Audio is outside wwwroot and is available
/// only to the staff member who saved it. Each review save creates a new revision.
/// Configure RsrStudy:ArchivePath on persistent storage when running in containers.
/// </summary>
[ApiController]
[Authorize(Roles = "admin,Admin,researcher,Researcher")]
[Route("api/rsr-study/archives")]
public class RsrStudyController : ControllerBase
{
    private readonly string _root;
    private readonly ILogger<RsrStudyController> _logger;

    public RsrStudyController(IConfiguration config, IWebHostEnvironment env, ILogger<RsrStudyController> logger)
    {
        _root = Path.GetFullPath(config["RsrStudy:ArchivePath"]
            ?? Path.Combine(env.ContentRootPath, "App_Data", "rsr-study"));
        _logger = logger;
    }

    private string? OwnerDirectory()
    {
        var owner = User.FindFirstValue(ClaimTypes.NameIdentifier);
        if (string.IsNullOrEmpty(owner)) return null;
        return Path.Combine(_root, Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(owner))));
    }

    [HttpPost("{studyId:guid}")]
    [RequestSizeLimit(200_000_000)]
    [RequestFormLimits(MultipartBodyLengthLimit = 200_000_000, ValueLengthLimit = 4_000_000)]
    public async Task<IActionResult> Save(Guid studyId, [FromForm] string metadata, [FromForm] IFormFile audio,
        CancellationToken cancellationToken)
    {
        var owner = OwnerDirectory();
        if (owner == null) return Unauthorized();
        if (audio == null || audio.Length == 0) return BadRequest("A full session recording is required.");
        if (string.IsNullOrWhiteSpace(metadata) || metadata.Length > 4_000_000)
            return BadRequest("Session metadata is missing or too large.");
        try
        {
            using var document = JsonDocument.Parse(metadata);
            var root = document.RootElement;
            if (!root.TryGetProperty("id", out var id) || !Guid.TryParse(id.GetString(), out var parsedId) || parsedId != studyId)
                return BadRequest("Study ID does not match metadata.");
            if (!root.TryGetProperty("phase", out var phase) || phase.GetString() is not ("finished" or "interrupted"))
                return BadRequest("Only stopped recordings can be archived.");
            if (!root.TryGetProperty("events", out var events) || events.ValueKind != JsonValueKind.Array ||
                !root.TryGetProperty("attempts", out var attempts) || attempts.ValueKind != JsonValueKind.Array)
                return BadRequest("Event timeline and attempts are required.");
        }
        catch (JsonException) { return BadRequest("Invalid JSON metadata."); }
        catch (InvalidOperationException) { return BadRequest("Invalid metadata fields."); }

        var revision = Guid.NewGuid();
        var sessionDirectory = Path.Combine(owner, studyId.ToString());
        var pending = Path.Combine(sessionDirectory, $".pending-{revision}");
        var destination = Path.Combine(sessionDirectory, revision.ToString());
        Directory.CreateDirectory(pending);
        try
        {
            await using (var output = System.IO.File.Create(Path.Combine(pending, "audio")))
                await audio.CopyToAsync(output, cancellationToken);
            await System.IO.File.WriteAllTextAsync(Path.Combine(pending, "metadata.json"), metadata, cancellationToken);
            // Publish only once both files have finished writing. Never replace a revision.
            Directory.Move(pending, destination);
            return Ok(new { studyId, revision, savedAt = DateTime.UtcNow });
        }
        catch
        {
            if (Directory.Exists(pending)) Directory.Delete(pending, true);
            throw;
        }
    }

    [HttpGet]
    public async Task<IActionResult> List(CancellationToken cancellationToken)
    {
        var owner = OwnerDirectory();
        if (owner == null) return Unauthorized();
        var archives = new List<object>();
        if (!Directory.Exists(owner)) return Ok(archives);
        foreach (var sessionDirectory in Directory.EnumerateDirectories(owner))
        {
            if (!Guid.TryParse(Path.GetFileName(sessionDirectory), out var studyId)) continue;
            var latest = Directory.EnumerateDirectories(sessionDirectory)
                .Where(path => Guid.TryParse(Path.GetFileName(path), out _))
                .OrderByDescending(path => System.IO.File.GetLastWriteTimeUtc(Path.Combine(path, "metadata.json")))
                .FirstOrDefault();
            if (latest == null) continue;
            try
            {
                using var document = JsonDocument.Parse(await System.IO.File.ReadAllTextAsync(Path.Combine(latest, "metadata.json"), cancellationToken));
                archives.Add(new { studyId, revision = Path.GetFileName(latest), metadata = document.RootElement.Clone() });
            }
            catch (JsonException ex) { _logger.LogWarning(ex, "Unreadable RSR archive {StudyId}", studyId); }
        }
        return Ok(archives);
    }

    [HttpGet("{studyId:guid}/{revision:guid}/audio")]
    public IActionResult Audio(Guid studyId, Guid revision)
    {
        var owner = OwnerDirectory();
        if (owner == null) return Unauthorized();
        var path = Path.Combine(owner, studyId.ToString(), revision.ToString(), "audio");
        if (!System.IO.File.Exists(path)) return NotFound();
        return PhysicalFile(path, "application/octet-stream", $"{studyId}-audio", enableRangeProcessing: true);
    }
}
