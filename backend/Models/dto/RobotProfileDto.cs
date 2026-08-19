namespace bloom.Models.dto
{
    public class RobotProfileDto
    {
        public required string Nickname { get; set; }
        public required string PersonalityTrait { get; set; }
        public string? Catchphrase { get; set; }
        public string? ColorTheme { get; set; }
        public string? Voice { get; set; }
    }
}
