using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Data
{
    public class ArSrDbContext : DbContext
    {
        public DbSet<ArSrEnrollment> Enrollments { get; set; }
        public DbSet<ArSrSession> Sessions { get; set; }
        public DbSet<ArSrSentenceResult> SentenceResults { get; set; }
        public DbSet<ArSrRecording> Recordings { get; set; }

        public ArSrDbContext(DbContextOptions<ArSrDbContext> options) : base(options) { }

        protected override void OnModelCreating(ModelBuilder builder)
        {
            base.OnModelCreating(builder);

            builder.Entity<ArSrEnrollment>(entity =>
            {
                entity.ToTable("ArSr_Enrollments");

                entity.HasMany(e => e.Sessions)
                    .WithOne(s => s.Enrollment)
                    .HasForeignKey(s => s.EnrollmentId)
                    .OnDelete(DeleteBehavior.Cascade);
            });

            builder.Entity<ArSrSession>(entity =>
            {
                entity.ToTable("ArSr_Sessions");

                entity.HasMany(s => s.SentenceResults)
                    .WithOne(r => r.Session)
                    .HasForeignKey(r => r.SessionId)
                    .OnDelete(DeleteBehavior.Cascade);

                entity.HasMany(s => s.Recordings)
                    .WithOne(r => r.Session)
                    .HasForeignKey(r => r.SessionId)
                    .OnDelete(DeleteBehavior.Cascade);
            });

            builder.Entity<ArSrSentenceResult>(entity =>
            {
                entity.ToTable("ArSr_SentenceResults");
            });

            builder.Entity<ArSrRecording>(entity =>
            {
                entity.ToTable("ArSr_Recordings");
            });
        }
    }
}
