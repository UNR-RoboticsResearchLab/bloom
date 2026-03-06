using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class continuity : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "TotalSteps",
                table: "LessonProgresses");

            migrationBuilder.AddColumn<int>(
                name: "TotalSteps",
                table: "Lessons",
                type: "int",
                nullable: false,
                defaultValue: 0);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "TotalSteps",
                table: "Lessons");

            migrationBuilder.AddColumn<int>(
                name: "TotalSteps",
                table: "LessonProgresses",
                type: "int",
                nullable: false,
                defaultValue: 0);
        }
    }
}
