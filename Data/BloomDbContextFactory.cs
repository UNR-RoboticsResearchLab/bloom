

using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Design;
using Microsoft.Extensions.Options;
using Microsoft.Extensions.Configuration;

namespace bloom.Data
{
    class BloomDbContextFactory : IDesignTimeDbContextFactory<BloomDbContext>
    {
        public BloomDbContext CreateDbContext(string[] args)
        {
            var optionsBuilder = new DbContextOptionsBuilder<BloomDbContext>();
            var configuration = new ConfigurationBuilder()
                .SetBasePath(Directory.GetCurrentDirectory())
                .AddJsonFile("appsettings.json")
                .AddEnvironmentVariables()
                .Build();

            var connectionString = Environment.GetEnvironmentVariable("ASPNETCORE_ENVIRONMENT") == "Development" 
                ? configuration.GetConnectionString("DefaultConnection") 
                : configuration.GetConnectionString("ProductionConnection");
            optionsBuilder.UseMySql(connectionString,
                new MySqlServerVersion(new Version(11, 7, 2)),
                mySqlOptions => mySqlOptions.EnableRetryOnFailure(
                        maxRetryCount: 5,
                        maxRetryDelay: TimeSpan.FromSeconds(30),
                        errorNumbersToAdd: null
                    )); 

            return new BloomDbContext(optionsBuilder.Options);
        }
    }
}