


namespace bloom.Models.dto
{

    public class LoginDto
    {
        public string? Email { get; set; }
        public string? AccessId { get; set; }
        public required string Password { get; set; }
    }

    public class CreateAccountDto
    {
        public required string FullName { get; set; }
        public required string Email { get; set; }
        public required string Password { get; set; }
        public required string SelectedRole { get; set; }
        public required string UserName { get; set; }
    }
    
}
