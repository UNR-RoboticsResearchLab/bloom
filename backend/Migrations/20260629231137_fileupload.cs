using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class fileupload : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<string>(
                name: "Title",
                table: "LessonSteps",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.AddColumn<string>(
                name: "Gaze",
                table: "Behaviors",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.AddColumn<string>(
                name: "HeadMovement",
                table: "Behaviors",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Title",
                table: "LessonSteps");

            migrationBuilder.DropColumn(
                name: "Gaze",
                table: "Behaviors");

            migrationBuilder.DropColumn(
                name: "HeadMovement",
                table: "Behaviors");
        }
    }
}
