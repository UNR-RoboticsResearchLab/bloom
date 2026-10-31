using System.Net;
using System.Reflection;
using System.Text;
using bloom.Models;
using bloom.Services.Accounts;
using bloom.Services.ArSr;
using bloom.Services.Classrooms;
using bloom.Services.Cors;
using bloom.Services.Lessons;
using bloom.Services.Robots;
using bloom.Services.Rsr;
using bloom.Services.SLPClients;
using bloom.Services.SessionControl;
using bloom.Data;
using bloom.Repositories;
using bloom.Services.RemoteShell;
using bloom.Services.ReleaseCatalog;
using Microsoft.Extensions.DependencyInjection.Extensions;
using Microsoft.AspNetCore.Authentication.Cookies;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.AspNetCore.OpenApi;
using Microsoft.Extensions.Options;
using Pomelo.EntityFrameworkCore.MySql.Internal;
using Microsoft.AspNetCore.DataProtection;
using System.Security.Cryptography.X509Certificates;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.IdentityModel.Tokens;
using bloom.Services.Email;
using Microsoft.AspNetCore.HttpOverrides;
using Microsoft.AspNetCore.RateLimiting;
using System.Threading.RateLimiting;

// Load .env (if present) into process environment variables for local development.
// Must run before WebApplication.CreateBuilder, which snapshots env vars into IConfiguration.
// Existing environment variables always win, so this never overrides real deployment config.
var envFilePath = Path.Combine(Directory.GetCurrentDirectory(), ".env");
if (File.Exists(envFilePath))
{
    foreach (var line in File.ReadAllLines(envFilePath))
    {
        var trimmed = line.Trim();
        if (trimmed.Length == 0 || trimmed.StartsWith('#'))
            continue;

        var separatorIndex = trimmed.IndexOf('=');
        if (separatorIndex <= 0)
            continue;

        var key = trimmed[..separatorIndex].Trim();
        var value = trimmed[(separatorIndex + 1)..].Trim().Trim('"');

        if (Environment.GetEnvironmentVariable(key) is null)
            Environment.SetEnvironmentVariable(key, value);
    }
}

var builder = WebApplication.CreateBuilder(args);

// Get ConnectionString
var build_environmment = builder.Environment.EnvironmentName;
var ConnectionString = build_environmment == "Production"
        ? builder.Configuration.GetConnectionString("ProductionConnection")
        : builder.Configuration.GetConnectionString("DefaultConnection");
var ArSrConnectionString = build_environmment == "Production"
        ? builder.Configuration.GetConnectionString("ProdArsrConnection")
        : builder.Configuration.GetConnectionString("DevArsrConnection");

var redactedConnectionString = System.Text.RegularExpressions.Regex.Replace(
    ConnectionString ?? "", @"(?i)(password|pwd)=[^;]*", "$1=***");
Console.WriteLine($"ConnectionString: {redactedConnectionString}");

//  ============ Add services to the container. ============


if (build_environmment == "Development")
{
    builder.Services.AddOpenApi();
    builder.Services.AddEndpointsApiExplorer();
    builder.Services.AddSwaggerGen(options =>
    {
        var xmlFilename = $"{Assembly.GetExecutingAssembly().GetName().Name}.xml";
        options.IncludeXmlComments(Path.Combine(AppContext.BaseDirectory, xmlFilename));
    });
    // builder.WebHost.ConfigureKestrel(options =>
    // {
    //     options.ListenAnyIP(8080);   // HTTP
    //     options.ListenAnyIP(2443, listenOptions => listenOptions.UseHttps()); // HTTPS optional
    // });

    // Unencrypted (no cert needed in dev), but still persisted to the dpkeys volume so
    // container restarts/rebuilds don't invalidate every logged-in cookie. Without this,
    // keys default to the container's ephemeral filesystem and every restart forces a
    // silent re-login (localStorage still says "logged in", every [Authorize] call 401s).
    var devKeyringPath = builder.Configuration["DataProtection:KeyringPath"] ?? "/var/dpkeys";
    builder.Services.AddDataProtection()
        .PersistKeysToFileSystem(new DirectoryInfo(devKeyringPath))
        .SetApplicationName("BloomServer");
}
else
{
    var certPath = builder.Configuration["DataProtection:CertPath"]
        ?? "/etc/bloom/certs/bloomserver.pfx";
    var certPassword = builder.Configuration["DataProtection:CertPassword"]
        ?? throw new InvalidOperationException(
            "DataProtection:CertPassword is not configured. Set the DataProtection__CertPassword environment variable.");
    var keyringPath = builder.Configuration["DataProtection:KeyringPath"]
        ?? "/var/dpkeys";

    var cert = X509CertificateLoader.LoadPkcs12FromFile(certPath, certPassword);

    builder.Services.AddDataProtection()
        .PersistKeysToFileSystem(new DirectoryInfo(keyringPath))
        .SetApplicationName("BloomServer")
        .SetDefaultKeyLifetime(TimeSpan.FromDays(90))
        .ProtectKeysWithCertificate(cert);
}

// Add DB Context
builder.Services.AddDbContext<BloomDbContext>(options =>
    options.UseMySql(ConnectionString,
        new MySqlServerVersion(new Version(11, 7, 2)),
        mySqlOptions => mySqlOptions.EnableRetryOnFailure(
                maxRetryCount: 5,
                maxRetryDelay: TimeSpan.FromSeconds(30),
                errorNumbersToAdd: null

    )));


// arsr db context
builder.Services.AddDbContext<ArSrDbContext>(options =>
    options.UseMySql(ArSrConnectionString,
        new MySqlServerVersion(new Version(11, 7, 2)),
        mySqlOptions => mySqlOptions.EnableRetryOnFailure(
                maxRetryCount: 5,
                maxRetryDelay: TimeSpan.FromSeconds(30),
                errorNumbersToAdd: null
    )));

// ArSr services
builder.Services.AddScoped<IArSrService, ArSrService>();
builder.Services.AddScoped<ArSrTranscriptionService>();
builder.Services.AddHttpClient("ArSrMicroservice", client =>
{
    client.BaseAddress = new Uri(builder.Configuration["ArSr:MicroserviceUrl"]
        ?? "http://arsr-service:5050");
    client.Timeout = TimeSpan.FromMinutes(5); // WhisperX can take time on long sessions
});


// Add identity
builder.Services.AddIdentity<Account, IdentityRole>(options =>
{
    options.Password.RequireDigit = true;
    options.Password.RequiredLength = 6;
    options.Password.RequireNonAlphanumeric = false;
    options.SignIn.RequireConfirmedAccount = true;
    options.User.RequireUniqueEmail = true;
})
.AddEntityFrameworkStores<BloomDbContext>()
.AddDefaultTokenProviders();

// Email confirmation links are valid for 24 hours.
builder.Services.Configure<DataProtectionTokenProviderOptions>(options =>
    options.TokenLifespan = TimeSpan.FromHours(24));

// Outgoing email (SMTP) and confirmation links
builder.Services.Configure<SmtpOptions>(builder.Configuration.GetSection(SmtpOptions.SectionName));
builder.Services.AddHttpContextAccessor();
builder.Services.AddScoped<IEmailSender, SmtpEmailSender>();
builder.Services.AddScoped<IEmailConfirmationService, EmailConfirmationService>();
builder.Services.AddScoped<IPasswordResetService, PasswordResetService>();

// Behind a TLS reverse proxy the request arrives as plain HTTP from the proxy; honor X-Forwarded-*
// so generated links and cookies use the public scheme and host. The proxy is not known by address
// (it may be a container), so known networks are cleared; the server port is only published on
// loopback / the proxy network (see docker-compose.prod.yml).
builder.Services.Configure<ForwardedHeadersOptions>(options =>
{
    options.ForwardedHeaders = ForwardedHeaders.XForwardedFor
        | ForwardedHeaders.XForwardedProto
        | ForwardedHeaders.XForwardedHost;
    options.KnownNetworks.Clear();
    options.KnownProxies.Clear();
});

// Limits how often confirmation / password reset emails can be requested (per client IP).
builder.Services.AddRateLimiter(options =>
{
    options.RejectionStatusCode = StatusCodes.Status429TooManyRequests;
    foreach (var policy in new[] { "resend-confirmation", "forgot-password" })
        options.AddPolicy(policy, httpContext =>
            RateLimitPartition.GetFixedWindowLimiter(
                policy + ":" + (httpContext.Connection.RemoteIpAddress?.ToString() ?? "unknown"),
                _ => new FixedWindowRateLimiterOptions
                {
                    PermitLimit = 3,
                    Window = TimeSpan.FromMinutes(10),
                    QueueLimit = 0
                }));
});

// Identity's application cookie: return 401/403 for API calls instead of redirecting to a login page.
// Also relax cookie security so LAN dev (cross-origin, plain HTTP) can carry the cookie.
//
// SameSite=None + SecurePolicy.Always looks like it should allow this, but does the
// opposite: None requires Secure, and browsers only transmit a Secure cookie over
// plain HTTP on origins they treat as "potentially trustworthy" -- localhost/127.0.0.1
// only, NOT a LAN IP like http://192.168.x.x. So the cookie silently never comes back
// on any machine other than the one running Docker, and every [Authorize] endpoint
// 401s (e.g. GET /api/user/clients, POST session join for robot pairing).
// Frontend and backend here are always same host/different port (same-site, just
// cross-origin), so SameSite=Lax is sufficient -- it only restricts cross-site
// requests, not same-site ones. SecurePolicy.SameAsRequest makes the cookie Secure
// when served over HTTPS (prod) and plain when served over HTTP (LAN dev), so it
// actually gets sent back in both cases.
builder.Services.ConfigureApplicationCookie(options =>
{
    options.Cookie.Name = "bloom_cookie";
    options.Cookie.HttpOnly = true;
    options.Cookie.SameSite = SameSiteMode.Lax;
    options.Cookie.SecurePolicy = CookieSecurePolicy.SameAsRequest;

    options.Events.OnRedirectToLogin = ctx =>
    {
        if (ctx.Request.Path.StartsWithSegments("/api") || ctx.Request.Path.StartsWithSegments("/hubs"))
        {
            ctx.Response.StatusCode = StatusCodes.Status401Unauthorized;
            return Task.CompletedTask;
        }
        ctx.Response.Redirect(ctx.RedirectUri);
        return Task.CompletedTask;
    };
    options.Events.OnRedirectToAccessDenied = ctx =>
    {
        if (ctx.Request.Path.StartsWithSegments("/api") || ctx.Request.Path.StartsWithSegments("/hubs"))
        {
            ctx.Response.StatusCode = StatusCodes.Status403Forbidden;
            return Task.CompletedTask;
        }
        ctx.Response.Redirect(ctx.RedirectUri);
        return Task.CompletedTask;
    };
});

// AddAuthentication() without a default scheme preserves Identity's cookie as the default.
// JWT is available via the "JwtOnly" / "JwtOrCookie" policies below.
builder.Services.AddAuthentication()
    .AddJwtBearer(options =>
    {
        options.TokenValidationParameters = new TokenValidationParameters
        {
            ValidateIssuer = false,
            ValidateAudience = false,
            ValidateLifetime = true,
            ValidateIssuerSigningKey = true,
            IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(
                builder.Configuration["Jwt:Key"]
                    ?? throw new InvalidOperationException("Jwt:Key is not configured.")
            ))
        };

        // The backend only mints robot JWTs (RobotService.GenerateRobotJwt), so anything else is
        // forged. Robot tokens must also carry the robot's current TokenVersion, which is what
        // makes rotate/revoke take effect immediately instead of at expiry.
        options.Events = new JwtBearerEvents
        {
            OnTokenValidated = async ctx =>
            {
                if (ctx.Principal?.IsInRole("Robot") != true)
                {
                    ctx.Fail("Unrecognized token.");
                    return;
                }
                var robots = ctx.HttpContext.RequestServices.GetRequiredService<IRobotService>();
                if (!await robots.IsRobotTokenCurrentAsync(ctx.Principal))
                    ctx.Fail("Robot token has been revoked.");
            }
        };
    });

// =========== Add Custom Services ===========
builder.Services.AddScoped<IAccountService, AccountService>();
builder.Services.AddScoped<IRobotService, RobotService>();
builder.Services.AddScoped<IRobotEventService, RobotEventService>();
builder.Services.AddScoped<IRobotProfileService, RobotProfileService>();
builder.Services.AddScoped<ILessonService, LessonService>();
builder.Services.AddScoped<ILessonProgressService, LessonProgressService>();
builder.Services.AddScoped<IAssignmentService, AssignmentService>();

// Lesson AI assistant — calls Azure OpenAI for lesson/step generation
builder.Services.AddScoped<ILessonAiService, LessonAiService>();
builder.Services.AddHttpClient("AzureOpenAILesson", client =>
{
    var endpoint = builder.Configuration["AZURE_OPENAI_ENDPOINT"]
        ?? throw new InvalidOperationException("AZURE_OPENAI_ENDPOINT is not configured.");
    client.BaseAddress = new Uri(endpoint);
    client.DefaultRequestHeaders.Add("api-key", builder.Configuration["AZURE_OPENAI_KEY"]
        ?? throw new InvalidOperationException("AZURE_OPENAI_KEY is not configured."));
    client.Timeout = TimeSpan.FromSeconds(60);
});

// Add RobotSession Services and Repositories
builder.Services.AddSingleton<IRobotStateRepository, InMemoryRobotStateRepository>();
builder.Services.AddScoped<IRobotSessionRepository, RobotSessionRepository>();
builder.Services.AddScoped<ISessionCodeService, SessionCodeService>();
builder.Services.AddScoped<IRobotSessionService, RobotSessionService>();
builder.Services.AddScoped<IRobotStateService, RobotStateService>();
builder.Services.AddScoped<ISLPClientService, SLPClientService>();
builder.Services.AddSingleton<IStepControlService, StepControlService>();
builder.Services.AddSingleton<IRepeatRequestDetector, RepeatRequestDetector>();
builder.Services.AddSingleton<IIdleModeService, IdleModeService>();
builder.Services.AddSingleton<IVoiceOverrideService, VoiceOverrideService>();
builder.Services.AddSingleton<IRsrSpeechService, RsrSpeechService>();

// Fleet remote shell. ShellRelay is a Singleton (process-wide tunnel/session map, like
// InMemoryRobotStateRepository); the connection handlers live for one WebSocket request each.
var remoteShellSection = builder.Configuration.GetSection(RemoteShellOptions.SectionName);
var remoteShellEnabled = remoteShellSection.Get<RemoteShellOptions>()?.Enabled == true;
builder.Services.Configure<RemoteShellOptions>(remoteShellSection);
builder.Services.TryAddSingleton(TimeProvider.System);
builder.Services.AddSingleton<IShellRelay, ShellRelay>();
builder.Services.AddScoped<IShellAuditService, ShellAuditService>();
// SignalR carries both legs. Registered even when the shell is off: the relay (and so the robot
// controller) takes IHubContext, which only needs the services, not the mapped endpoints.
{
    var shell = remoteShellSection.Get<RemoteShellOptions>() ?? new RemoteShellOptions();
    builder.Services.AddSignalR(o =>
    {
        o.MaximumReceiveMessageSize = 256 * 1024;  // the default 32 KB is smaller than a big paste
        o.KeepAliveInterval = TimeSpan.FromSeconds(15);
        // Must be at least twice the keepalive. A silent client past this is dropped, which is how a
        // robot that vanished without a FIN (Wi-Fi gone) is noticed.
        o.ClientTimeoutInterval = TimeSpan.FromSeconds(Math.Max(shell.AgentIdleTimeoutSeconds, 30));
    });
}

if (remoteShellEnabled)
{
    // The default Jwt:Key is checked into git. With it, anyone can mint a robot token -- fine for
    // poking at lesson endpoints in dev, not for a sudo shell on every robot.
    var jwtKey = builder.Configuration["Jwt:Key"] ?? "";
    if (jwtKey == "ThisIsASecretKeyForJwtTokenGeneration" || Encoding.UTF8.GetByteCount(jwtKey) < 32)
        throw new InvalidOperationException(
            "RemoteShell:Enabled requires a private Jwt:Key of at least 32 bytes. Set the Jwt__Key environment variable.");
}

// AutoRSR microservice integration
var autoRsrBaseUrl = builder.Configuration["AutoRsr:BaseUrl"] ?? "http://localhost:5050";
builder.Services.AddHttpClient("AutoRsr", client =>
{
    client.BaseAddress = new Uri(autoRsrBaseUrl);
    client.Timeout = TimeSpan.FromMinutes(10);
});
builder.Services.AddScoped<IRsrService, RsrService>();

// OTA release list for the fleet view: versions from config plus the GitHub Releases of a repo.
builder.Services.Configure<ReleaseCatalogOptions>(builder.Configuration.GetSection(ReleaseCatalogOptions.SectionName));
builder.Services.AddMemoryCache();
builder.Services.AddHttpClient(ReleaseCatalogService.HttpClientName, client =>
{
    client.BaseAddress = new Uri("https://api.github.com/");
    client.Timeout = TimeSpan.FromSeconds(10);
    client.DefaultRequestHeaders.UserAgent.ParseAdd("bloom-backend");  // GitHub rejects requests without one
    client.DefaultRequestHeaders.Accept.ParseAdd("application/vnd.github+json");
    client.DefaultRequestHeaders.Add("X-GitHub-Api-Version", "2022-11-28");
});
builder.Services.AddSingleton<IReleaseCatalogService, ReleaseCatalogService>();

// Add MVC model
builder.Services.AddControllersWithViews();

builder.Services.AddSession(options =>
{
    options.IdleTimeout = TimeSpan.FromMinutes(30);
    options.Cookie.HttpOnly = true;
    options.Cookie.IsEssential = true;
    options.Cookie.SameSite = SameSiteMode.Lax;  // or None+Secure if you really need cross-site
});

// Enable CORS for development
// TODO: add production check
// localhost + private-LAN addresses are allowed; anything else (e.g. a Tailscale address a dev frontend is
// opened from) must be listed in Cors:AllowedOrigins, comma separated (env: Cors__AllowedOrigins).
var extraCorsOrigins = CorsOrigins.ParseList(builder.Configuration["Cors:AllowedOrigins"]);
builder.Services.AddCors(options => {
    options.AddDefaultPolicy(policy => {
        policy
            .SetIsOriginAllowed(origin => CorsOrigins.IsAllowed(origin, extraCorsOrigins))
            .AllowAnyHeader()
            .AllowAnyMethod()
            .AllowCredentials();
    });
});


builder.Services.AddAuthorization(options =>
{
    options.AddPolicy("JwtOnly", policy => policy
        .AddAuthenticationSchemes(JwtBearerDefaults.AuthenticationScheme)
        .RequireAuthenticatedUser());

    options.AddPolicy("CookieOnly", policy => policy
        .AddAuthenticationSchemes(IdentityConstants.ApplicationScheme)
        .RequireAuthenticatedUser());

    options.AddPolicy("JwtOrCookie", policy => policy
        .AddAuthenticationSchemes(JwtBearerDefaults.AuthenticationScheme, IdentityConstants.ApplicationScheme)
        .RequireAuthenticatedUser());

    // Fleet administration: provisioning, revocation, fleet view, remote shell. Cookie only on
    // purpose -- the backend only ever mints Robot JWTs, so a JWT claiming Admin is forged by
    // definition and must not be able to satisfy this.
    // Robot agent tunnel: only tokens from a RobotId + Secret login (never legacy name-only ones).
    options.AddPolicy("RobotAgent", policy => policy
        .AddAuthenticationSchemes(JwtBearerDefaults.AuthenticationScheme)
        .RequireRole("Robot")
        .RequireClaim(RobotService.CredentialClaim, "secret")
        .RequireClaim(RobotService.TokenVersionClaim));

    options.AddPolicy("AdminOnly", policy => policy
        .AddAuthenticationSchemes(IdentityConstants.ApplicationScheme)
        .RequireRole("Admin"));
});

var app = builder.Build();

// ============ Configure the HTTP request pipeline. ============
if (app.Environment.IsDevelopment())
{
    // The default HSTS value is 30 days. You may want to change this for production scenarios, see https://aka.ms/aspnetcore-hsts.
    app.MapOpenApi();
    app.UseSwagger();
    app.UseSwaggerUI();
}
else
{
    app.UseHsts();
}

// app.UseHttpsRedirection();
app.UseForwardedHeaders();
app.UseCors();
app.UseMiddleware<ShellHubOriginCheck>();
app.UseDefaultFiles();
app.UseStaticFiles();

app.UseRouting();
app.UseRateLimiter();
app.UseSession();
app.UseAuthentication();
app.UseAuthorization();



using (var scope = app.Services.CreateScope())
{
    var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();

    // WebApplicationFactory-based integration tests (tests/Integration/BloomWebApplicationFactory.cs)
    // swap this DbContext to an in-memory SQLite connection. The checked-in EF Core migrations
    // were generated against MySQL/Pomelo and contain MySQL-specific SQL, so Migrate() throws
    // under SQLite. EnsureCreated() builds the schema directly from the current model instead
    // (no migration history involved) -- provider-agnostic and sufficient for tests. Real
    // deployments (MySQL) are unaffected: this branch is purely additive to the test path.
    if (db.Database.ProviderName == "Microsoft.EntityFrameworkCore.Sqlite")
        db.Database.EnsureCreated();
    else
        db.Database.Migrate();

    var roleManager = scope.ServiceProvider.GetRequiredService<RoleManager<IdentityRole>>();
    await BloomDbContext.SeedDatabaseRoles(roleManager);

    var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
    // Check if admin account exists, if not create one
    await BloomDbContext.SeedDatabaseAdminUser(
        userManager, app.Configuration["Seed:AdminPassword"], app.Environment.IsProduction());

    await LessonSeeder.SeedLessonsFromFilesAsync(db, userManager);
}

app.MapControllers();

if (remoteShellEnabled)
{
    // The agent speaks only WebSockets (it connects without SignalR's negotiate step); browsers get
    // SignalR's normal transport fallback.
    app.MapHub<RobotAgentHub>("/hubs/robot-agent",
            o => o.Transports = Microsoft.AspNetCore.Http.Connections.HttpTransportType.WebSockets)
        .RequireAuthorization("RobotAgent");
    app.MapHub<ShellHub>("/hubs/shell").RequireAuthorization("AdminOnly");
}

app.MapFallbackToFile("index.html");

app.Run();


public partial class Program
{
    
}