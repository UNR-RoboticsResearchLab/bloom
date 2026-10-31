using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class AddRobotEvents : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.CreateTable(
                name: "RobotEvents",
                columns: table => new
                {
                    Id = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci"),
                    RobotId = table.Column<Guid>(type: "char(36)", nullable: false, collation: "ascii_general_ci"),
                    RobotSessionId = table.Column<Guid>(type: "char(36)", nullable: true, collation: "ascii_general_ci"),
                    EventType = table.Column<string>(type: "varchar(32)", maxLength: 32, nullable: false)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    Severity = table.Column<int>(type: "int", nullable: false),
                    Source = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    Code = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    Message = table.Column<string>(type: "longtext", nullable: false)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    Details = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4"),
                    OccurredAt = table.Column<DateTime>(type: "datetime(6)", nullable: false),
                    ReceivedAt = table.Column<DateTime>(type: "datetime(6)", nullable: false),
                    IsResolved = table.Column<bool>(type: "tinyint(1)", nullable: false),
                    ResolvedAt = table.Column<DateTime>(type: "datetime(6)", nullable: true),
                    ResolutionNotes = table.Column<string>(type: "longtext", nullable: true)
                        .Annotation("MySql:CharSet", "utf8mb4")
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_RobotEvents", x => x.Id);
                    table.ForeignKey(
                        name: "FK_RobotEvents_RobotSessions_RobotSessionId",
                        column: x => x.RobotSessionId,
                        principalTable: "RobotSessions",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.SetNull);
                    table.ForeignKey(
                        name: "FK_RobotEvents_Robots_RobotId",
                        column: x => x.RobotId,
                        principalTable: "Robots",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Cascade);
                })
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_RobotEvents_RobotId_OccurredAt",
                table: "RobotEvents",
                columns: new[] { "RobotId", "OccurredAt" });

            migrationBuilder.CreateIndex(
                name: "IX_RobotEvents_RobotSessionId",
                table: "RobotEvents",
                column: "RobotSessionId");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropTable(
                name: "RobotEvents");
        }
    }
}
