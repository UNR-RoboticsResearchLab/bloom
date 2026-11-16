using System.ComponentModel.DataAnnotations;
using System.Security.Claims;
using System.Text.Json;
using bloom.Data;
using bloom.Models;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;

namespace bloom.Controllers;

[ApiController]
[Route("api/rsr-study/animation-behaviors")]
[Authorize(Policy = "CookieOnly", Roles = "Admin,admin")]
public class RsrAnimationBehaviorsController(BloomDbContext db) : ControllerBase
{
    [HttpPost]
    [Consumes("application/json")]
    [RequestSizeLimit(256 * 1024)]
    public async Task<IActionResult> Save(SaveAnimationBehavior request, CancellationToken ct)
    {
        var owner = User.FindFirstValue(ClaimTypes.NameIdentifier);
        if (string.IsNullOrEmpty(owner)) return Forbid();
        if (request.Script.ValueKind != JsonValueKind.Array ||
            request.Expressions.ValueKind != JsonValueKind.Array ||
            request.Script.GetArrayLength() + request.Expressions.GetArrayLength() == 0 ||
            request.Script.EnumerateArray().Any(item => !ValidKeyframe(item)) ||
            request.Expressions.EnumerateArray().Any(item => !ValidExpression(item)))
            return BadRequest(new { message = "Provide a valid motor or expression timeline." });

        var name = request.Name.Trim();
        var behavior = new RsrAnimationBehavior {
            Id = Guid.NewGuid(), Name = name, CreatedById = owner, CreatedAtUtc = DateTime.UtcNow,
            PayloadJson = JsonSerializer.Serialize(new { name, script = request.Script, expressions = request.Expressions })
        };
        db.RsrAnimationBehaviors.Add(behavior);
        await db.SaveChangesAsync(ct);
        return CreatedAtAction(nameof(Download), new { id = behavior.Id },
            new { behavior.Id, behavior.Name, behavior.CreatedAtUtc });
    }

    [HttpGet]
    public async Task<IActionResult> List(CancellationToken ct) => Ok(await db.RsrAnimationBehaviors
        .AsNoTracking().OrderByDescending(row => row.CreatedAtUtc).Take(100)
        .Select(row => new { row.Id, row.Name, row.CreatedAtUtc }).ToListAsync(ct));

    [HttpGet("{id:guid}")]
    public async Task<IActionResult> Download(Guid id, CancellationToken ct)
    {
        var row = await db.RsrAnimationBehaviors.AsNoTracking().SingleOrDefaultAsync(row => row.Id == id, ct);
        if (row == null) return NotFound();
        Response.Headers.CacheControl = "no-store";
        return File(System.Text.Encoding.UTF8.GetBytes(row.PayloadJson), "application/json", $"behavior-{id}.json");
    }

    private static bool Nonnegative(JsonElement item, string field) =>
        item.TryGetProperty(field, out var value) && value.ValueKind == JsonValueKind.Number &&
        value.TryGetDouble(out var number) && double.IsFinite(number) && number >= 0;

    private static bool ValidKeyframe(JsonElement item) =>
        item.ValueKind == JsonValueKind.Object && Nonnegative(item, "at") && Nonnegative(item, "duration") &&
        item.TryGetProperty("pose", out var pose) && pose.ValueKind == JsonValueKind.Object &&
        pose.EnumerateObject().Any() && pose.EnumerateObject().All(p =>
            p.Value.ValueKind == JsonValueKind.Number && p.Value.TryGetDouble(out var angle) && double.IsFinite(angle));

    private static bool ValidExpression(JsonElement item) =>
        item.ValueKind == JsonValueKind.Object && Nonnegative(item, "at") && Nonnegative(item, "duration") &&
        item.TryGetProperty("name", out var name) && name.ValueKind == JsonValueKind.String &&
        !string.IsNullOrWhiteSpace(name.GetString());
}

public class SaveAnimationBehavior
{
    [Required, StringLength(120)] public string Name { get; set; } = "";
    public JsonElement Script { get; set; }
    public JsonElement Expressions { get; set; }
}
