using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class robotMigration : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Robots_Accounts_RegisteredUserId",
                table: "Robots");

            migrationBuilder.AddForeignKey(
                name: "FK_Robots_Accounts_RegisteredUserId",
                table: "Robots",
                column: "RegisteredUserId",
                principalTable: "Accounts",
                principalColumn: "Id",
                onDelete: ReferentialAction.SetNull);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Robots_Accounts_RegisteredUserId",
                table: "Robots");

            migrationBuilder.AddForeignKey(
                name: "FK_Robots_Accounts_RegisteredUserId",
                table: "Robots",
                column: "RegisteredUserId",
                principalTable: "Accounts",
                principalColumn: "Id");
        }
    }
}
