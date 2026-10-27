using bloom.Models;
using System.Security.Claims;
using Microsoft.AspNetCore.Identity;

namespace bloom.Services;

/// <summary>One-shot server-side bootstrap; never promotes or resets an existing account.</summary>
public static class StudyAccountProvisioner
{
    public const string PasswordChangeClaim = "study:must-change-password";

    public static async Task CreateAsync(UserManager<Account> users, string login, string password, string role)
    {
        login = login.Trim();
        if (string.IsNullOrWhiteSpace(login) || string.IsNullOrEmpty(password) ||
            (role != "Admin" && role != "Researcher"))
            throw new InvalidOperationException("Provide a login, password, and Admin or Researcher role.");
        if (await users.FindByNameAsync(login) != null)
            throw new InvalidOperationException("The login is already assigned; no changes were made.");
        var user = new Account { UserName = login, Role = role, CreatedDate = DateTime.UtcNow };
        var created = await users.CreateAsync(user, password);
        if (!created.Succeeded)
            throw new InvalidOperationException(string.Join(" ", created.Errors.Select(e => e.Description)));
        var assigned = await users.AddToRoleAsync(user, role);
        if (assigned.Succeeded) assigned = await users.AddClaimAsync(user, new Claim(PasswordChangeClaim, "true"));
        if (!assigned.Succeeded)
        {
            await users.DeleteAsync(user);
            throw new InvalidOperationException("Could not assign study access.");
        }
    }
}
