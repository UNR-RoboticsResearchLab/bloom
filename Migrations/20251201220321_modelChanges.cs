using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class modelChanges : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<Guid>(
                name: "ClassroomId",
                table: "Lessons",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

            migrationBuilder.AddColumn<Guid>(
                name: "ClassroomId",
                table: "Assignments",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

            migrationBuilder.CreateIndex(
                name: "IX_Lessons_ClassroomId",
                table: "Lessons",
                column: "ClassroomId");

            migrationBuilder.CreateIndex(
                name: "IX_Assignments_ClassroomId",
                table: "Assignments",
                column: "ClassroomId");

            migrationBuilder.AddForeignKey(
                name: "FK_Assignments_Classrooms_ClassroomId",
                table: "Assignments",
                column: "ClassroomId",
                principalTable: "Classrooms",
                principalColumn: "Id");

            migrationBuilder.AddForeignKey(
                name: "FK_Lessons_Classrooms_ClassroomId",
                table: "Lessons",
                column: "ClassroomId",
                principalTable: "Classrooms",
                principalColumn: "Id");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Assignments_Classrooms_ClassroomId",
                table: "Assignments");

            migrationBuilder.DropForeignKey(
                name: "FK_Lessons_Classrooms_ClassroomId",
                table: "Lessons");

            migrationBuilder.DropIndex(
                name: "IX_Lessons_ClassroomId",
                table: "Lessons");

            migrationBuilder.DropIndex(
                name: "IX_Assignments_ClassroomId",
                table: "Assignments");

            migrationBuilder.DropColumn(
                name: "ClassroomId",
                table: "Lessons");

            migrationBuilder.DropColumn(
                name: "ClassroomId",
                table: "Assignments");
        }
    }
}
