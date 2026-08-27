using System;
using System.Linq;
using bloom.Data;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.DataProtection;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Data.Sqlite;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.Extensions.DependencyInjection;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// WebApplicationFactory for controller-level integration tests. Swaps BloomDbContext to a
    /// SQLite connection held open for the lifetime of this factory instance -- a bare
    /// "DataSource=:memory:" string opens a brand-new, empty in-memory database every time EF
    /// opens a connection, so nothing would persist across the many DbContext scopes one HTTP
    /// request creates. Holding one open SqliteConnection and passing that object to UseSqlite
    /// keeps them all talking to the same in-memory database for as long as this factory lives.
    ///
    /// One instance of this factory is created per test CLASS via IClassFixture&lt;&gt;, so each
    /// test class gets its own isolated database AND its own instances of the services
    /// registered as DI singletons in Program.cs (IStepControlService, IRobotStateRepository,
    /// IRepeatRequestDetector) -- none of that state crosses test-class boundaries. Tests WITHIN
    /// one class share both the database and those singletons, so each test method should seed
    /// its own fresh data (fresh Guids/emails via TestDataSeeder) rather than relying on
    /// anything set up by a different test in the same class.
    /// </summary>
    public class BloomWebApplicationFactory : WebApplicationFactory<Program>
    {
        private readonly SqliteConnection _connection;

        public BloomWebApplicationFactory()
        {
            // Program.cs's top-level code branches on EnvironmentName before this factory's
            // ConfigureWebHost hook runs (WebApplicationFactory patches services into the
            // builder before Build() is called, but everything between Build() and Run() --
            // including that branch -- executes for real). Setting this here, before
            // WebApplication.CreateBuilder(args) ever reads it, guarantees the Development
            // branch is taken (persisted DataProtection keys via a cert, required in the
            // "else" branch, would throw on a CI runner with no cert/CertPassword configured).
            Environment.SetEnvironmentVariable("ASPNETCORE_ENVIRONMENT", "Development");

            _connection = new SqliteConnection("DataSource=:memory:");
            _connection.Open();
        }

        protected override void ConfigureWebHost(IWebHostBuilder builder)
        {
            builder.UseEnvironment("Development");

            builder.ConfigureServices(services =>
            {
                // Removing just the DbContextOptions<BloomDbContext> descriptor isn't enough:
                // since EF Core 5, AddDbContext also registers the provider-configuring action
                // itself as an IDbContextOptionsConfiguration<BloomDbContext> descriptor (to
                // support layering multiple AddDbContext/ConfigureDbContext calls), and that
                // registration is what actually attaches UseMySql(...) from Program.cs's
                // original registration. Leaving it behind means BOTH Pomelo/MySQL's and
                // SQLite's provider get attached to the rebuilt options, and EF throws
                // "Only a single database provider can be registered" the first time anything
                // touches the DbContext. Remove both kinds of descriptor before re-registering.
                var descriptorsToRemove = services.Where(d =>
                    d.ServiceType == typeof(DbContextOptions<BloomDbContext>) ||
                    (d.ServiceType.IsGenericType &&
                     d.ServiceType.GetGenericTypeDefinition() == typeof(IDbContextOptionsConfiguration<>) &&
                     d.ServiceType.GetGenericArguments()[0] == typeof(BloomDbContext))
                ).ToList();

                foreach (var descriptor in descriptorsToRemove)
                {
                    services.Remove(descriptor);
                }

                // Add SQLite in-memory, backed by the connection held open for this factory's lifetime.
                services.AddDbContext<BloomDbContext>(options =>
                    options.UseSqlite(_connection));

                // Program.cs's Development branch persists DataProtection keys to
                // DataProtection:KeyringPath (default /var/dpkeys), which won't exist/won't be
                // writable on a CI runner. Ephemeral keys are fine here -- this factory (and any
                // cookies it issues) never outlives one test class.
                services.AddDataProtection().UseEphemeralDataProtectionProvider();
            });
        }

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
            if (disposing)
            {
                _connection.Dispose();
            }
        }
    }
}
