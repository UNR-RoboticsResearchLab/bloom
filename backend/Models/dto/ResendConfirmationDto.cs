using System.ComponentModel.DataAnnotations;

namespace bloom.Models.dto
{
    public class ResendConfirmationDto
    {
        [Required]
        [EmailAddress]
        public string Email { get; set; } = "";
    }
}
