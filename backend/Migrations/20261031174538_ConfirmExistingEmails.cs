using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <summary>
    /// Data-only. Sign-in now requires a confirmed email, but every account that exists before this
    /// release was created with EmailConfirmed = false and would be locked out. Mark them confirmed.
    /// </summary>
    public partial class ConfirmExistingEmails : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.Sql("UPDATE `Accounts` SET `EmailConfirmed` = 1 WHERE `EmailConfirmed` = 0;");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            // The previous state is not recoverable (it was "unconfirmed" for everyone), so this is a no-op.
        }
    }
}
