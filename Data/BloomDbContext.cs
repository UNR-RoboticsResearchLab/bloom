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


            

            // configure facilitatoruser relationships
            
            builder.Entity<FacilitatorUser>()
                .HasMany(s => s.Students)
                .WithOne()
                .HasForeignKey(s => s.CreatedById);
            builder.Entity<FacilitatorUser>()
                .HasMany(l => l.Lessons)
                .WithOne()
                .HasForeignKey(l => l.CreatedById);
            

            // configure studentuser relationships
            builder.Entity<StudentUser>()
                .HasMany(a => a.Assignments)
                .WithOne(a => a.Student)
                .HasForeignKey(a => a.StudentId);

            builder.Entity<StudentUser>()
                .HasOne(a => a.CreatedBy)
                .WithMany()
                .HasForeignKey(s => s.CreatedById)
                .OnDelete(DeleteBehavior.Restrict);


            //configure lesson relationships
            builder.Entity<Lesson>()
                .HasOne(a => a.CreatedBy)
                .WithMany()
                .HasForeignKey(s => s.CreatedById)
                .OnDelete(DeleteBehavior.Restrict);



            //configure classroom relationship
            

    


        }
    }
}