// bloom
// BloomDbContext.cs
// File provinding a Db context for the BloomDb

using Bloom.Models;
using Microsoft.EntityFrameworkCore;
using Microsoft.AspNetCore.Identity;
using Microsoft.AspNetCore.Identity.EntityFrameworkCore;

namespace Bloom.Data
{
    public class BloomDbContext : IdentityDbContext<Account>
    {

        public BloomDbContext(DbContextOptions dbContextOptions) : base(dbContextOptions)
        {

        }
        
        protected override void OnModelCreating (ModelBuilder builder)
        {
            base.OnModelCreating(builder);

            //configure tables
            builder.Entity<Account>(entity => { entity.ToTable("Accounts"); });
            
            builder.Entity<StudentUser>(entity => { entity.ToTable("StudentUsers"); });
            builder.Entity<AdminUser>(entity => { entity.ToTable("AdminUsers"); });
            builder.Entity<FacilitatorUser>(entity => { entity.ToTable("FacilitatorUsers"); });


            // configure adminuser relationships
            builder.Entity<AdminUser>()
                .HasMany(s => s.Students)
                .WithOne()
                .HasForeignKey(s => s.CreatedById);

            builder.Entity<AdminUser>()
                .HasMany(l => l.Lessons)
                .WithOne()
                .HasForeignKey(l => l.CreatedById);

            builder.Entity<AdminUser>()
                .HasMany(a => a.Assignments)
                .WithOne()
                .HasForeignKey(a => a.AssignedById);

            // configure facilitatoruser relationships
            
            builder.Entity<FacilitatorUser>()
                .HasMany(s => s.Students)
                .WithOne()
                .HasForeignKey(s => s.CreatedById);
            builder.Entity<FacilitatorUser>()
                .HasMany(l => l.Lessons)
                .WithOne()
                .HasForeignKey(l => l.CreatedById);
            
            builder.Entity<FacilitatorUser>()
                .HasMany(a => a.Assignments)
                .WithOne()
                .HasForeignKey(a => a.AssignedById);
            
            // configure studentuser relationships

        }
    }
}