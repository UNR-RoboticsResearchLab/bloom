using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class AddLessonVisibilityAndLineage : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<Guid>(
                name: "AdaptedFromLessonId",
                table: "Lessons",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

            migrationBuilder.AddColumn<bool>(
                name: "IsPublic",
                table: "Lessons",
                type: "tinyint(1)",
                nullable: false,
                defaultValue: true);

            migrationBuilder.CreateIndex(
                name: "IX_Lessons_AdaptedFromLessonId",
                table: "Lessons",
                column: "AdaptedFromLessonId");

            migrationBuilder.AddForeignKey(
                name: "FK_Lessons_Lessons_AdaptedFromLessonId",
                table: "Lessons",
                column: "AdaptedFromLessonId",
                principalTable: "Lessons",
                principalColumn: "Id",
                onDelete: ReferentialAction.SetNull);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Lessons_Lessons_AdaptedFromLessonId",
                table: "Lessons");

            migrationBuilder.DropIndex(
                name: "IX_Lessons_AdaptedFromLessonId",
                table: "Lessons");

            migrationBuilder.DropColumn(
                name: "AdaptedFromLessonId",
                table: "Lessons");

            migrationBuilder.DropColumn(
                name: "IsPublic",
                table: "Lessons");
        }
    }
}
