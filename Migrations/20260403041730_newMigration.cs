using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class newMigration : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.RenameColumn(
                name: "StudentResponse",
                table: "LessonInteractions",
                newName: "DialogTurn");

            migrationBuilder.AlterColumn<string>(
                name: "InteractionType",
                table: "LessonInteractions",
                type: "longtext",
                nullable: true,
                oldClrType: typeof(string),
                oldType: "longtext")
                .Annotation("MySql:CharSet", "utf8mb4")
                .OldAnnotation("MySql:CharSet", "utf8mb4");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.RenameColumn(
                name: "DialogTurn",
                table: "LessonInteractions",
                newName: "StudentResponse");

            migrationBuilder.UpdateData(
                table: "LessonInteractions",
                keyColumn: "InteractionType",
                keyValue: null,
                column: "InteractionType",
                value: "");

            migrationBuilder.AlterColumn<string>(
                name: "InteractionType",
                table: "LessonInteractions",
                type: "longtext",
                nullable: false,
                oldClrType: typeof(string),
                oldType: "longtext",
                oldNullable: true)
                .Annotation("MySql:CharSet", "utf8mb4")
                .OldAnnotation("MySql:CharSet", "utf8mb4");
        }
    }
}
