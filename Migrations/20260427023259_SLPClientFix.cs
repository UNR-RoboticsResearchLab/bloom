using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations
{
    /// <inheritdoc />
    public partial class SLPClientFix : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.Sql("UPDATE `Assignments` SET `SLPClientId` = NULL WHERE `SLPClientId` IS NOT NULL;");
            migrationBuilder.Sql("DELETE FROM `SLPClients`;");

            migrationBuilder.DropForeignKey(
                name: "FK_Accounts_SLPClients_SLPClientId",
                table: "Accounts");

            migrationBuilder.DropForeignKey(
                name: "FK_SLPClients_Accounts_StudentId",
                table: "SLPClients");

            migrationBuilder.DropIndex(
                name: "IX_Accounts_SLPClientId",
                table: "Accounts");

            migrationBuilder.DropColumn(
                name: "SLPClientId",
                table: "Accounts");

            migrationBuilder.AddColumn<string>(
                name: "SlpId",
                table: "SLPClients",
                type: "varchar(255)",
                nullable: false,
                defaultValue: "")
                .Annotation("MySql:CharSet", "utf8mb4");

            migrationBuilder.CreateIndex(
                name: "IX_SLPClients_SlpId",
                table: "SLPClients",
                column: "SlpId");

            migrationBuilder.AddForeignKey(
                name: "FK_SLPClients_Accounts_SlpId",
                table: "SLPClients",
                column: "SlpId",
                principalTable: "Accounts",
                principalColumn: "Id",
                onDelete: ReferentialAction.Restrict);

            migrationBuilder.AddForeignKey(
                name: "FK_SLPClients_Accounts_StudentId",
                table: "SLPClients",
                column: "StudentId",
                principalTable: "Accounts",
                principalColumn: "Id",
                onDelete: ReferentialAction.Restrict);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_SLPClients_Accounts_SlpId",
                table: "SLPClients");

            migrationBuilder.DropForeignKey(
                name: "FK_SLPClients_Accounts_StudentId",
                table: "SLPClients");

            migrationBuilder.DropIndex(
                name: "IX_SLPClients_SlpId",
                table: "SLPClients");

            migrationBuilder.DropColumn(
                name: "SlpId",
                table: "SLPClients");

            migrationBuilder.AddColumn<Guid>(
                name: "SLPClientId",
                table: "Accounts",
                type: "char(36)",
                nullable: true,
                collation: "ascii_general_ci");

            migrationBuilder.CreateIndex(
                name: "IX_Accounts_SLPClientId",
                table: "Accounts",
                column: "SLPClientId");

            migrationBuilder.AddForeignKey(
                name: "FK_Accounts_SLPClients_SLPClientId",
                table: "Accounts",
                column: "SLPClientId",
                principalTable: "SLPClients",
                principalColumn: "Id");

            migrationBuilder.AddForeignKey(
                name: "FK_SLPClients_Accounts_StudentId",
                table: "SLPClients",
                column: "StudentId",
                principalTable: "Accounts",
                principalColumn: "Id",
                onDelete: ReferentialAction.Cascade);
        }
    }
}
