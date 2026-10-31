using System;
using System.IO;
using System.Linq;
using bloom.Data;
using bloom.Services.Email;
using bloom.Tests.TestHelpers;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.DataProtection;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Data.Sqlite;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.DependencyInjection.Extensions;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// WebApplicationFactory for controller-level integration tests. Swaps BloomDbContext to a SQLite
    /// database file private to this factory instance, in WAL mode with a busy timeout.
    ///
    /// Why a file rather than ":memory:": every in-memory connection is its own empty database, so
    /// tests had to share ONE held-open connection -- which SQLite doesn't support using from several
    /// threads at once. Plain HTTP tests never did, but the remote shell hubs do (a disconnect handler
    /// writes an audit row while a test polls for it), which showed up as intermittent "database is
    /// locked". With a file each DbContext gets its own connection; WAL lets readers proceed during a
    /// write and the busy timeout makes a second writer wait instead of failing.
    ///
    /// One instance of this factory is created per test CLASS via IClassFixture&lt;&gt;, so each
    /// test class gets its own isolated database AND its own instances of the services
    /// registered as DI singletons in Program.cs (IStepControlService, IRobotStateRepository,
    /// IRepeatRequestDetector, IShellRelay) -- none of that state crosses test-class boundaries. Tests
    /// WITHIN one class share both the database and those singletons, so each test method should seed
    /// its own fresh data (fresh Guids/emails via TestDataSeeder) rather than relying on anything set
    /// up by a different test in the same class.
    /// </summary>
    public class BloomWebApplicationFactory : WebApplicationFactory<Program>
    {
        private readonly string _databasePath;
        private readonly string _connectionString;

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

            // Program.cs also reads these at top level: RemoteShell:Enabled decides whether the
            // /ws endpoints are mapped, and refuses the checked-in default Jwt:Key.
            Environment.SetEnvironmentVariable("RemoteShell__Enabled", "true");
            Environment.SetEnvironmentVariable("RemoteShell__SecretPushTimeoutSeconds", "2");
            Environment.SetEnvironmentVariable("Jwt__Key", "integration-tests-only-signing-key-0123456789abcdef");

            _databasePath = Path.Combine(Path.GetTempPath(), $"bloom-tests-{Guid.NewGuid():N}.db");
            _connectionString = new SqliteConnectionStringBuilder
            {
                DataSource = _databasePath,
                Pooling = false,          // so Dispose can delete the file, and no connection outlives its DbContext
                DefaultTimeout = 30,      // also sets SQLite's busy timeout: a second writer waits
            }.ToString();

            // WAL is a property of the file, so setting it once here covers every later connection.
            using var setup = new SqliteConnection(_connectionString);
            setup.Open();
            using var wal = setup.CreateCommand();
            wal.CommandText = "PRAGMA journal_mode=WAL;";
            wal.ExecuteNonQuery();
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

                services.AddDbContext<BloomDbContext>(options =>
                    options.UseSqlite(_connectionString));

                // Program.cs's Development branch persists DataProtection keys to
                // DataProtection:KeyringPath (default /var/dpkeys), which won't exist/won't be
                // writable on a CI runner. Ephemeral keys are fine here -- this factory (and any
                // cookies it issues) never outlives one test class.
                services.AddDataProtection().UseEphemeralDataProtectionProvider();

                // Never talk to a real SMTP server from tests; capture the mail instead.
                services.RemoveAll<IEmailSender>();
                services.AddSingleton<FakeEmailSender>();
                services.AddSingleton<IEmailSender>(sp => sp.GetRequiredService<FakeEmailSender>());
            });
        }

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
            if (disposing)
            {
                SqliteConnection.ClearAllPools();
                foreach (var suffix in new[] { "", "-wal", "-shm" })
                    File.Delete(_databasePath + suffix);
            }
        }
    }
}
