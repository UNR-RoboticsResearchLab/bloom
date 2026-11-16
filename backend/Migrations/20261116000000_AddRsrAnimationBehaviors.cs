using bloom.Data;
using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace bloom.Migrations;

[DbContext(typeof(BloomDbContext))]
[Migration("20261116000000_AddRsrAnimationBehaviors")]
public class AddRsrAnimationBehaviors : Migration
{
    protected override void Up(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.CreateTable(
            name: "RsrAnimationBehaviors",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "char(36)", nullable: false),
                CreatedAtUtc = table.Column<DateTime>(type: "datetime(6)", nullable: false),
                CreatedById = table.Column<string>(type: "varchar(255)", maxLength: 255, nullable: false),
                Name = table.Column<string>(type: "varchar(120)", maxLength: 120, nullable: false),
                PayloadJson = table.Column<string>(type: "longtext", nullable: false)
            },
            constraints: table => table.PrimaryKey("PK_RsrAnimationBehaviors", row => row.Id))
            .Annotation("MySql:CharSet", "utf8mb4");
    }

    protected override void Down(MigrationBuilder migrationBuilder) =>
        migrationBuilder.DropTable(name: "RsrAnimationBehaviors");
}
