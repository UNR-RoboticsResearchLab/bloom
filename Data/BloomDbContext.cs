// bloom
// BloomDbContext.cs
// File providing a Db context for the BloomDb
// Created: 10/22/2025

using bloom.Models;
using Microsoft.EntityFrameworkCore;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Identity.EntityFrameworkCore;
using System.Text.Json;

namespace bloom.Data
{
    public class BloomDbContext : IdentityDbContext<Account>
    {
        public DbSet<Account> Accounts { get; set; }
        public DbSet<Lesson> Lessons { get; set; }
        public DbSet<Assignment> Assignments { get; set; }
        public DbSet<SLPClient> SLPClients { get; set; }
        public DbSet<Robot> Robots { get; set; }
        public DbSet<RobotSession> RobotSessions { get; set; }
        public DbSet<RobotStateHistory> RobotStateHistorys { get; set; }
        public DbSet<LessonProgress> LessonProgresses { get; set; }

        public BloomDbContext(DbContextOptions dbContextOptions) : base(dbContextOptions)
        {
            
            
        }

        protected override void OnModelCreating(ModelBuilder builder)
        {
            base.OnModelCreating(builder);

            // Configure tables
            builder.Entity<Account>(entity => { entity.ToTable("Accounts"); });

            builder.Entity<Robot>(entity => { entity.ToTable("Robots"); });

            builder.Entity<Lesson>(entity =>
            {
                entity.ToTable("Lessons");
                entity.HasOne(l => l.CreatedBy)
                    .WithMany(a => a.CreatedLessons)
                    .HasForeignKey(l => l.CreatedById)
                    .OnDelete(DeleteBehavior.Restrict);

                entity.HasMany(l => l.Assignments)
                    .WithOne(a => a.Lesson)
                    .HasForeignKey(a => a.LessonId)
                    .OnDelete(DeleteBehavior.Cascade);
            });

            builder.Entity<Assignment>(entity =>
            {
                entity.ToTable("Assignments");

                entity.HasOne(a => a.Student)
                    .WithMany(s => s.AssignedAssignments)
                    .HasForeignKey(a => a.StudentId)
                    .OnDelete(DeleteBehavior.Cascade);

                entity.HasOne(a => a.AssignedBy)
                    .WithMany()
                    .HasForeignKey(a => a.AssignedById)
                    .OnDelete(DeleteBehavior.Restrict);
            });

            builder.Entity<SLPClient>(entity =>
            {
                entity.ToTable("SLPClients");

                // One-to-Many: SLPClient - Student (Account)
                entity.HasOne(c => c.Student)
                    .WithMany()
                    .HasForeignKey(c => c.StudentId)
                    .OnDelete(DeleteBehavior.Cascade);

                // Many-to-Many: SLPClient - Teachers (Accounts)
                entity.HasMany(c => c.Teachers)
                    .WithMany();

                // Many-to-Many: SLPClient - Lessons
                entity.HasMany(c => c.Lessons)
                    .WithMany();

                // Many-to-Many: SLPClient - Assignments
                entity.HasMany(c => c.Assignments)
                    .WithMany();
            });

            builder.Entity<RobotSession>(entity =>
            {
                entity.ToTable("RobotSessions");

                entity.HasOne(rs => rs.User)
                    .WithMany()
                    .HasForeignKey(rs => rs.UserId)
                    .OnDelete(DeleteBehavior.SetNull);
            });

            builder.Entity<RobotStateHistory>(entity =>
            {
                entity.ToTable("RobotStateHistories");

                entity.HasOne(r => r.RobotSession)
                    .WithMany(s => s.StateHistory)
                    .HasForeignKey(r => r.RobotSessionId)
                    .OnDelete(DeleteBehavior.Cascade);

                // Configure RobotState as an owned entity (since it's marked with [Owned])
                entity.OwnsOne(r => r.RobotState);
            });

            builder.Entity<Robot>(entity => { 
                entity.HasOne(r => r.RegisteredUser)
                    .WithMany(a => a.RegisteredRobots)
                    .HasForeignKey(r => r.RegisteredUserId)
                    .OnDelete(DeleteBehavior.SetNull);
            });

        }

        public static async Task SeedDatabaseRoles(RoleManager<IdentityRole> roleMgr)
        {
            string[] roleNames = { "Admin", "SLP", "Student" };

            foreach (var roleName in roleNames)
            {
                if (!await roleMgr.RoleExistsAsync(roleName))
                    await roleMgr.CreateAsync(new IdentityRole(roleName));
            }
        }

        public static async Task SeedDatabaseAdminUser(UserManager<Account> userMgr)
        {
            // grab environment variables for admin user
            string adminEmail = "admin@example.com";
            string adminPassword = "Admin@123";
            if (await userMgr.FindByEmailAsync(adminEmail) == null)
            {
                var adminUser = new Account
                {
                    UserName = adminEmail,
                    Email = adminEmail,
                    FullName = "System Administrator",
                    CreatedDate = DateTime.UtcNow,
                    Role = "Admin"
                };

                var result = await userMgr.CreateAsync(adminUser, adminPassword);
                if (result.Succeeded)
                {
                    await userMgr.AddToRoleAsync(adminUser, "Admin");
                }
            }
        }
    }
}