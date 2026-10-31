namespace bloom.Models.dto
{
    public class AdminUserDto
    {
        public required string Id { get; set; }
        public string? FullName { get; set; }
        public string? Email { get; set; }
        public string? UserName { get; set; }

        // The user's Identity role (what authorization uses); falls back to Account.Role if none.
        public string? Role { get; set; }
        public DateTime CreatedDate { get; set; }
    }

    public class SetUserRoleDto
    {
        public required string Role { get; set; }
    }
}
