using bloom.Data;
using bloom.Tests.Integration;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc.Testing;


namespace bloom.Tests.Integration;

public class BloomWebApplicationFactory : BloomWebApplicationFactory
{
    
    protected override void ConfigureWebHost(IWebHostBuilder builder )
    {
        builder.ConfigureServices (services =>
        {
            // get rid of MQL
            var descriptor = services.SingleOrDefault(
                d => d.ServiceType == typeof(DbContextOptions<BloomDbContext>)
            );

            if (descriptor != null)
            {
                services.Remove(descriptor);
            }

            // Add SQLite in-memory
            services.AddDbContext<BloomDbContext>(options =>
                options.UseSqlite("DataSource=:memory:"));

            // build a temp service provider for db
            var sp = services.BuildServiceProvider();
            using var scope = sp.CreateScope();
            var  db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();

            //

        });
    }
}