using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class AddBehaviors : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_RobotStateHistories_Behavior_RobotState_CurrentBehaviorId",
                table: "RobotStateHistories");

            migrationBuilder.DropPrimaryKey(
                name: "PK_Behavior",
                table: "Behavior");

            migrationBuilder.DropColumn(
                name: "Behaviors",
                table: "LessonSteps");

            migrationBuilder.DropColumn(
                name: "LearningObjectives",
                table: "Lessons");

            migrationBuilder.RenameTable(
                name: "Behavior",
                newName: "Behaviors");

            migrationBuilder.AddPrimaryKey(
                name: "PK_Behaviors",
                table: "Behaviors",
                column: "Id");

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

            migrationBuilder.AddForeignKey(
                name: "FK_RobotStateHistories_Behaviors_RobotState_CurrentBehaviorId",
                table: "RobotStateHistories",
                column: "RobotState_CurrentBehaviorId",
                principalTable: "Behaviors",
                principalColumn: "Id");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_RobotStateHistories_Behaviors_RobotState_CurrentBehaviorId",
                table: "RobotStateHistories");

            migrationBuilder.DropTable(
                name: "LessonStepBehaviors");

            migrationBuilder.DropPrimaryKey(
                name: "PK_Behaviors",
                table: "Behaviors");

            migrationBuilder.RenameTable(
                name: "Behaviors",
                newName: "Behavior");

            migrationBuilder.AddColumn<string>(
                name: "Behaviors",
                table: "LessonSteps",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.AddColumn<string>(
                name: "LearningObjectives",
                table: "Lessons",
                type: "longtext",
                nullable: true)
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.AddPrimaryKey(
                name: "PK_Behavior",
                table: "Behavior",
                column: "Id");

            migrationBuilder.AddForeignKey(
                name: "FK_RobotStateHistories_Behavior_RobotState_CurrentBehaviorId",
                table: "RobotStateHistories",
                column: "RobotState_CurrentBehaviorId",
                principalTable: "Behavior",
                principalColumn: "Id");
        }
    }
}
