using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class FixBehaviors : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropTable(
                name: "LessonStepBehaviors");

            migrationBuilder.AddColumn<int>(
                name: "BehaviorId",
                table: "LessonSteps",
                type: "int",
                nullable: true);

            migrationBuilder.AddColumn<string>(
                name: "FacialExpression",
                table: "Behaviors",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_LessonSteps_BehaviorId",
                table: "LessonSteps",
                column: "BehaviorId");

            migrationBuilder.AddForeignKey(
                name: "FK_LessonSteps_Behaviors_BehaviorId",
                table: "LessonSteps",
                column: "BehaviorId",
                principalTable: "Behaviors",
                principalColumn: "Id",
                onDelete: ReferentialAction.SetNull);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_LessonSteps_Behaviors_BehaviorId",
                table: "LessonSteps");

            migrationBuilder.DropIndex(
                name: "IX_LessonSteps_BehaviorId",
                table: "LessonSteps");

            migrationBuilder.DropColumn(
                name: "BehaviorId",
                table: "LessonSteps");

            migrationBuilder.DropColumn(
                name: "FacialExpression",
                table: "Behaviors");

            migrationBuilder.CreateTable(
                name: "LessonStepBehaviors",
                columns: table => new
                {
                    BehaviorsId = table.Column<int>(type: "int", nullable: false),
                    LessonStepId = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci")
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_LessonStepBehaviors", x => new { x.BehaviorsId, x.LessonStepId });
                    table.ForeignKey(
                        name: "FK_LessonStepBehaviors_Behaviors_BehaviorsId",
                        column: x => x.BehaviorsId,
                        principalTable: "Behaviors",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Cascade);
                    table.ForeignKey(
                        name: "FK_LessonStepBehaviors_LessonSteps_LessonStepId",
                        column: x => x.LessonStepId,
                        principalTable: "LessonSteps",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Cascade);
                })
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_LessonStepBehaviors_LessonStepId",
                table: "LessonStepBehaviors",
                column: "LessonStepId");
        }
    }
}
