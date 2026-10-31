namespace bloom.Models.dto
{
    /// <summary>
    /// The versions an admin can push to a robot, per release channel, newest first.
    /// </summary>
    public class ReleaseCatalogDto
    {
        // "prod", "canary" and "dev" are always present, possibly empty.
        public required Dictionary<string, List<string>> Channels { get; set; }

        // Sources that couldn't be read (e.g. GitHub unreachable); the lists hold what could.
        public List<string> Warnings { get; set; } = new();
    }
}
