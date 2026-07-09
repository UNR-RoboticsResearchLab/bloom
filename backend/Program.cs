using System.Net;
using System.Reflection;
using System.Text;
using bloom.Models;
using bloom.Services;
using bloom.Data;
using bloom.Repositories;
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
})
.AddEntityFrameworkStores<BloomDbContext>()
.AddDefaultTokenProviders();

// Identity's application cookie: return 401/403 for API calls instead of redirecting to a login page.
// Also relax cookie security so LAN dev (cross-origin, plain HTTP) can carry the cookie.
builder.Services.ConfigureApplicationCookie(options =>
{
    options.Cookie.Name = "bloom_cookie";
    options.Cookie.HttpOnly = true;
    options.Cookie.SameSite = SameSiteMode.None;
    options.Cookie.SecurePolicy = CookieSecurePolicy.Always;

    options.Events.OnRedirectToLogin = ctx =>
    {
        if (ctx.Request.Path.StartsWithSegments("/api"))
        {
            ctx.Response.StatusCode = StatusCodes.Status401Unauthorized;
            return Task.CompletedTask;
        }
        ctx.Response.Redirect(ctx.RedirectUri);
        return Task.CompletedTask;
    };
    options.Events.OnRedirectToAccessDenied = ctx =>
    {
        if (ctx.Request.Path.StartsWithSegments("/api"))
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
    });

// =========== Add Custom Services ===========
builder.Services.AddScoped<IAccountService, AccountService>();
builder.Services.AddScoped<IRobotService, RobotService>();
builder.Services.AddScoped<ILessonService, LessonService>();
builder.Services.AddScoped<ILessonProgressService, LessonProgressService>();

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
builder.Services.AddSingleton<IRsrSpeechService, RsrSpeechService>();

// AutoRSR microservice integration
var autoRsrBaseUrl = builder.Configuration["AutoRsr:BaseUrl"] ?? "http://localhost:5050";
builder.Services.AddHttpClient("AutoRsr", client =>
{
    client.BaseAddress = new Uri(autoRsrBaseUrl);
    client.Timeout = TimeSpan.FromMinutes(10);
});
builder.Services.AddScoped<IRsrService, RsrService>();

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
builder.Services.AddCors(options => {
    options.AddDefaultPolicy(policy => {
        policy
            .SetIsOriginAllowed(origin => {
                var uri = new Uri(origin);
                // allow localhost + RFC1918 LAN ranges in dev
                return uri.Host == "localhost"
                    || uri.Host.StartsWith("127.")
                    || uri.Host.StartsWith("192.168.")
                    || uri.Host.StartsWith("10.")
                    || (uri.Host.StartsWith("172.") && int.TryParse(uri.Host.Split('.')[1], out var o) && o >= 16 && o <= 31);
            })
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
app.UseCors();
app.UseDefaultFiles();
app.UseStaticFiles();

app.UseRouting();
app.UseSession();
app.UseAuthentication();
app.UseAuthorization();



using (var scope = app.Services.CreateScope())
{
    var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();
    db.Database.Migrate();

    var roleManager = scope.ServiceProvider.GetRequiredService<RoleManager<IdentityRole>>();
    await BloomDbContext.SeedDatabaseRoles(roleManager);

    var userManager = scope.ServiceProvider.GetRequiredService<UserManager<Account>>();
    // Check if admin account exists, if not create one
    await BloomDbContext.SeedDatabaseAdminUser(userManager);

    await LessonSeeder.SeedLessonsFromFilesAsync(db, userManager);
}

app.MapControllers();

app.MapFallbackToFile("index.html");

app.Run();


public partial class Program
{
    
}