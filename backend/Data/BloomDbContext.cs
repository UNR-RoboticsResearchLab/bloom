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
        public DbSet<Classroom> Classrooms { get; set; }
        public DbSet<Robot> Robots { get; set; }
        public DbSet<RobotSession> RobotSessions { get; set; }
        public DbSet<RobotStateHistory> RobotStateHistorys { get; set; }
        public DbSet<LessonStep> LessonSteps { get; set; }
        public DbSet<LessonProgress> LessonProgresses { get; set; }
        public DbSet<LessonInteraction> LessonInteractions { get; set; }
        public DbSet<LessonRun> LessonRuns { get; set; }
        public DbSet<StepInteraction> StepInteractions { get; set; }
        public DbSet<SLPClient> SLPClients { get; set; }
        public DbSet<RsrAssessment> RsrAssessments { get; set; }
        public DbSet<Behavior> Behaviors { get; set; }

        public BloomDbContext(DbContextOptions<BloomDbContext> dbContextOptions) : base(dbContextOptions)
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

                entity.HasMany(l => l.Steps)
                    .WithOne(s => s.Lesson)
                    .HasForeignKey(s => s.LessonId)
                    .OnDelete(DeleteBehavior.Cascade);
            });

            builder.Entity<LessonStep>(entity =>
            {
                entity.ToTable("LessonSteps");

                entity.HasOne(s => s.Interaction)
                    .WithOne(i => i.LessonStep)
                    .HasForeignKey<LessonStep>(s => s.InteractionId)
                    .OnDelete(DeleteBehavior.Cascade);

                entity.HasOne(s => s.Behaviors)
                    .WithMany()
                    .HasForeignKey(s => s.BehaviorId)
                    .OnDelete(DeleteBehavior.SetNull);
            });

            builder.Entity<StepInteraction>(entity =>
            {
                entity.ToTable("StepInteractions");
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

                entity.HasOne(c => c.Student)
                    .WithMany()
                    .HasForeignKey(c => c.StudentId)
                    .OnDelete(DeleteBehavior.Restrict);

                entity.HasOne(c => c.Slp)
                    .WithMany()
                    .HasForeignKey(c => c.SlpId)
                    .OnDelete(DeleteBehavior.Restrict);
            });

            builder.Entity<Classroom>(entity =>
            {
                entity.ToTable("Classrooms");

                // Many-to-Many: Classroom - Students (Accounts)
                entity.HasMany(c => c.Students)
                    .WithMany();

                // Many-to-Many: Classroom - Teachers (Accounts)
                entity.HasMany(c => c.Teachers)
                    .WithMany();
            });

            builder.Entity<RobotSession>(entity =>
            {
                entity.ToTable("RobotSessions");

                entity.HasOne(rs => rs.User)
                    .WithMany()
                    .HasForeignKey(rs => rs.UserId)
                    .OnDelete(DeleteBehavior.SetNull);

                entity.HasOne(rs => rs.ActiveLessonRun)
                    .WithMany()
                    .HasForeignKey(rs => rs.ActiveLessonRunId)
                    .OnDelete(DeleteBehavior.SetNull);
            });

            builder.Entity<LessonRun>(entity =>
            {
                entity.ToTable("LessonRuns");

                entity.HasOne(r => r.RobotSession)
                    .WithMany()
                    .HasForeignKey(r => r.RobotSessionId)
                    .OnDelete(DeleteBehavior.Cascade);

                entity.HasOne(r => r.Lesson)
                    .WithMany()
                    .HasForeignKey(r => r.LessonId)
                    .OnDelete(DeleteBehavior.Restrict);

                entity.HasMany(r => r.Interactions)
                    .WithOne(i => i.LessonRun)
                    .HasForeignKey(i => i.LessonRunId)
                    .OnDelete(DeleteBehavior.Cascade);

                entity.HasIndex(r => new { r.RobotSessionId, r.LessonId, r.StartedAt });
            });

            builder.Entity<LessonInteraction>(entity =>
            {
                entity.HasIndex(li => new { li.LessonRunId, li.InteractionType, li.IsAcknowledged });
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

            builder.Entity<RsrAssessment>(entity =>
            {
                entity.ToTable("RsrAssessments");
                entity.HasIndex(a => a.Pid).IsUnique();
            });

        }

        public static async Task SeedDatabaseRoles(RoleManager<IdentityRole> roleMgr)
        {
            string[] roleNames = { "Admin", "SLP", "Student", "Researcher", "Participant" };

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


            string slpEmail = "slp@example.com";
            string slpPassword = "Slp@123";

            if (await userMgr.FindByEmailAsync(slpEmail) == null)
            {
                var slpUser = new Account
                {
                    UserName = slpEmail,
                    Email = slpEmail,
                    FullName = "SLP Account",
                    CreatedDate = DateTime.UtcNow,
                    Role = "SLP"
                };
                var result = await userMgr.CreateAsync(slpUser, slpPassword);

                if (result.Succeeded)
                {
                    await userMgr.AddToRoleAsync(slpUser, "SLP");
                }
            }
        }

        public async Task SeedLessons()
        {
                if (!Lessons.Any())
                {
                    var sampleLessons = new List<Lesson>
                    {
                        new Lesson
                        {
                            Title = "Sample Language Lesson",
                            Description = "A sample language lesson for testing.",
                            CreatedDate = DateTime.UtcNow,
                            CreatedById = Accounts.FirstOrDefault()?.Id ?? Guid.NewGuid().ToString(),
                            LessonType = LessonType.Language,
                            TotalSteps = 5
                        },
                        new Lesson
                        {
                            Title = "Sample Speech Therapy Lesson",
                            Description = "A sample speech therapy lesson for testing.",
                            CreatedDate = DateTime.UtcNow,
                            CreatedById = Accounts.FirstOrDefault()?.Id ?? Guid.NewGuid().ToString(),
                            LessonType = LessonType.Speech,
                            TotalSteps = 7
                        }
                    };
    
                    Lessons.AddRange(sampleLessons);
                    await SaveChangesAsync();
                }
        }
    }
}