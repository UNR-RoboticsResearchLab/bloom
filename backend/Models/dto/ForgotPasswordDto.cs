using System.ComponentModel.DataAnnotations;

namespace bloom.Models.dto
{
    public class ForgotPasswordDto
    {
        [Required]
        [EmailAddress]
        public string Email { get; set; } = "";
    }
}
