


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
        public string FullName { get; set; }
        public string Email { get; set; }
        public string Password { get; set; }
        public string SelectedRole { get; set; }
        public string UserName { get; set; }
    }
    
}
