using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class AddUserToRobotSession : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<string>(
                name: "UserId",
                table: "RobotSessions",
                type: "varchar(255)",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_RobotSessions_UserId",
                table: "RobotSessions",
                column: "UserId");

            migrationBuilder.AddForeignKey(
                name: "FK_RobotSessions_Accounts_UserId",
                table: "RobotSessions",
                column: "UserId",
                principalTable: "Accounts",
                principalColumn: "Id",
                onDelete: ReferentialAction.SetNull);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_RobotSessions_Accounts_UserId",
                table: "RobotSessions");

            migrationBuilder.DropIndex(
                name: "IX_RobotSessions_UserId",
                table: "RobotSessions");

            migrationBuilder.DropColumn(
                name: "UserId",
                table: "RobotSessions");
        }
    }
}
