


namespace bloom.Models.dto
{

    public class LoginDto
    {
        public string? Email;
        public string? AccessId;
        public required string Password;
    }

    public class CreateAccountDto
    {
        public string FullName;
        public string Email;
        public string Password;
        public string SelectedRole;
        public string UserName;
    }
    
}
