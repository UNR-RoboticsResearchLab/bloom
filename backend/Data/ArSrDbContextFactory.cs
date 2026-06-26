using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Design;
using Microsoft.Extensions.Configuration;

namespace bloom.Data
{
    class ArSrDbContextFactory : IDesignTimeDbContextFactory<ArSrDbContext>
    {
        public ArSrDbContext CreateDbContext(string[] args)
        {
            var optionsBuilder = new DbContextOptionsBuilder<ArSrDbContext>();
            var configuration = new ConfigurationBuilder()
                .SetBasePath(Directory.GetCurrentDirectory())
                .AddJsonFile("appsettings.json")
                .AddEnvironmentVariables()
                .Build();

            var connectionString = Environment.GetEnvironmentVariable("ASPNETCORE_ENVIRONMENT") == "Development"
                ? configuration.GetConnectionString("DevArsrConnection")
                : configuration.GetConnectionString("ProdArsrConnection");
            optionsBuilder.UseMySql(connectionString,
                new MySqlServerVersion(new Version(11, 7, 2)),
                mySqlOptions => mySqlOptions.EnableRetryOnFailure(
                        maxRetryCount: 5,
                        maxRetryDelay: TimeSpan.FromSeconds(30),
                        errorNumbersToAdd: null
                    ));

            return new ArSrDbContext(optionsBuilder.Options);
        }
    }
}
