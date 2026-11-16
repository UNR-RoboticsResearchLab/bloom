using System.ComponentModel.DataAnnotations;

namespace bloom.Models;

public class RsrAnimationBehavior
{
    public Guid Id { get; set; }
    [MaxLength(120)] public required string Name { get; set; }
    [MaxLength(255)] public required string CreatedById { get; set; }
    public required string PayloadJson { get; set; }
    public DateTime CreatedAtUtc { get; set; }
}
