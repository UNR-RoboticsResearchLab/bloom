using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class AddInteraction : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Interaction",
                table: "LessonSteps");

            migrationBuilder.AddColumn<Guid>(
                name: "InteractionId",
                table: "LessonSteps",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

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

            migrationBuilder.CreateTable(
                name: "StepInteractions",
                columns: table => new
                {
                    Id = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci"),
                    WaitForResponse = table.Column<bool>(type: "tinyint(1)", nullable: false),
                    MaxWaitSeconds = table.Column<int>(type: "int", nullable: true),
                    CorrectAnswer = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    CorrectResponseScript = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    IncorrectResponseScript = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    SingleTurnLlm = table.Column<bool>(type: "tinyint(1)", nullable: false),
                    SingleTurnLlmPrompt = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    LlmFollowUp = table.Column<bool>(type: "tinyint(1)", nullable: false),
                    FallbackScript = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    FallbackVisualAid = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    FallbackVisualAidLabels = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4")
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_StepInteractions", x => x.Id);
                })
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_LessonSteps_InteractionId",
                table: "LessonSteps",
                column: "InteractionId",
                unique: true);

            migrationBuilder.AddForeignKey(
                name: "FK_LessonSteps_StepInteractions_InteractionId",
                table: "LessonSteps",
                column: "InteractionId",
                principalTable: "StepInteractions",
                principalColumn: "Id",
                onDelete: ReferentialAction.Cascade);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_LessonSteps_StepInteractions_InteractionId",
                table: "LessonSteps");

            migrationBuilder.DropTable(
                name: "StepInteractions");

            migrationBuilder.DropIndex(
                name: "IX_LessonSteps_InteractionId",
                table: "LessonSteps");

            migrationBuilder.DropColumn(
                name: "InteractionId",
                table: "LessonSteps");

            migrationBuilder.AddColumn<string>(
                name: "Interaction",
                table: "LessonSteps",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

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
    }
}
