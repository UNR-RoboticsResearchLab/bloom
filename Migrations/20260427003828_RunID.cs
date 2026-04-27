using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class RunID : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<Guid>(
                name: "ActiveLessonRunId",
                table: "RobotSessions",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

            migrationBuilder.AlterColumn<string>(
                name: "InteractionType",
                table: "LessonInteractions",
                type: "varchar(255)",
                nullable: false,
                oldClrType: typeof(string),
                oldType: "longtext")
                .Annotation("MySql:CharSet", "utf8mb4")
                .OldAnnotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.AddColumn<Guid>(
                name: "LessonRunId",
                table: "LessonInteractions",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

            migrationBuilder.CreateTable(
                name: "LessonRuns",
                columns: table => new
                {
                    Id = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci"),
                    RobotSessionId = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci"),
                    LessonId = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci"),
                    SlpId = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    StudentId = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    StartedAt = table.Column<DateTime>(type: "datetime(6)", nullable: false),
                    EndedAt = table.Column<DateTime>(type: "datetime(6)", nullable: true),
                    Status = table.Column<string>(type: "longtext", nullable: false)
                        .Annotation("MySql:CharSet", "utf8mb4")
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_LessonRuns", x => x.Id);
                    table.ForeignKey(
                        name: "FK_LessonRuns_Lessons_LessonId",
                        column: x => x.LessonId,
                        principalTable: "Lessons",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Restrict);
                    table.ForeignKey(
                        name: "FK_LessonRuns_RobotSessions_RobotSessionId",
                        column: x => x.RobotSessionId,
                        principalTable: "RobotSessions",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Cascade);
                })
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_RobotSessions_ActiveLessonRunId",
                table: "RobotSessions",
                column: "ActiveLessonRunId");

            migrationBuilder.CreateIndex(
                name: "IX_LessonInteractions_LessonRunId_InteractionType_IsAcknowledged",
                table: "LessonInteractions",
                columns: new[] { "LessonRunId", "InteractionType", "IsAcknowledged" });

            migrationBuilder.CreateIndex(
                name: "IX_LessonRuns_LessonId",
                table: "LessonRuns",
                column: "LessonId");

            migrationBuilder.CreateIndex(
                name: "IX_LessonRuns_RobotSessionId_LessonId_StartedAt",
                table: "LessonRuns",
                columns: new[] { "RobotSessionId", "LessonId", "StartedAt" });

            migrationBuilder.AddForeignKey(
                name: "FK_LessonInteractions_LessonRuns_LessonRunId",
                table: "LessonInteractions",
                column: "LessonRunId",
                principalTable: "LessonRuns",
                principalColumn: "Id",
                onDelete: ReferentialAction.Cascade);

            migrationBuilder.AddForeignKey(
                name: "FK_RobotSessions_LessonRuns_ActiveLessonRunId",
                table: "RobotSessions",
                column: "ActiveLessonRunId",
                principalTable: "LessonRuns",
                principalColumn: "Id",
                onDelete: ReferentialAction.SetNull);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_LessonInteractions_LessonRuns_LessonRunId",
                table: "LessonInteractions");

            migrationBuilder.DropForeignKey(
                name: "FK_RobotSessions_LessonRuns_ActiveLessonRunId",
                table: "RobotSessions");

            migrationBuilder.DropTable(
                name: "LessonRuns");

            migrationBuilder.DropIndex(
                name: "IX_RobotSessions_ActiveLessonRunId",
                table: "RobotSessions");

            migrationBuilder.DropIndex(
                name: "IX_LessonInteractions_LessonRunId_InteractionType_IsAcknowledged",
                table: "LessonInteractions");

            migrationBuilder.DropColumn(
                name: "ActiveLessonRunId",
                table: "RobotSessions");

            migrationBuilder.DropColumn(
                name: "LessonRunId",
                table: "LessonInteractions");

            migrationBuilder.AlterColumn<string>(
                name: "InteractionType",
                table: "LessonInteractions",
                type: "longtext",
                nullable: false,
                oldClrType: typeof(string),
                oldType: "varchar(255)")
                .Annotation("MySql:CharSet", "utf8mb4")
                .OldAnnotation("MySql:CharSet", "utf8mb4");
        }
    }
}
