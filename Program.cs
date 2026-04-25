using System.Net;
using System.Reflection;
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

var builder = WebApplication.CreateBuilder(args);

// Get ConnectionString
var build_environmment = builder.Environment.EnvironmentName;
var ConnectionString = build_environmment == "Production"
        ? builder.Configuration.GetConnectionString("ProductionConnection") 
        : builder.Configuration.GetConnectionString("DefaultConnection");

Console.WriteLine($"ConnectionString: {ConnectionString}");

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
    var cert = X509CertificateLoader.LoadPkcs12FromFile("certs/bloomserver.pfx", "bloomserver");

    builder.Services.AddDataProtection()
        .PersistKeysToFileSystem(new DirectoryInfo("/var/dpkeys"))
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




// Add identity
builder.Services.AddIdentity<Account, IdentityRole>(options =>
{
    options.Password.RequireDigit = true;
    options.Password.RequiredLength = 6;
    options.Password.RequireNonAlphanumeric = false;
})
.AddEntityFrameworkStores<BloomDbContext>()
.AddDefaultTokenProviders();

// =========== Add Custom Services ===========
builder.Services.AddScoped<IAccountService, AccountService>();
builder.Services.AddScoped<IRobotService, RobotService>();
builder.Services.AddScoped<ILessonProgressService, LessonProgressService>();

// Add RobotSession Services and Repositories
builder.Services.AddSingleton<IRobotStateRepository, InMemoryRobotStateRepository>();
builder.Services.AddScoped<IRobotSessionRepository, RobotSessionRepository>();
builder.Services.AddScoped<ISessionCodeService, SessionCodeService>();
builder.Services.AddScoped<IRobotSessionService, RobotSessionService>();
builder.Services.AddScoped<IRobotStateService, RobotStateService>();
builder.Services.AddScoped<ISLPClientService, SLPClientService>();
builder.Services.AddSingleton<IStepControlService, StepControlService>();

// Add MVC model
builder.Services.AddControllersWithViews();

// Add Cookie Auth
builder.Services.AddAuthentication(CookieAuthenticationDefaults.AuthenticationScheme)
    .AddCookie(options =>
    {
        options.LoginPath = builder.Configuration.GetValue<string>("LoginPath");
        options.LogoutPath = builder.Configuration.GetValue<string>("LogoutPath");
        options.Cookie.HttpOnly = true;

        //TODO: development comment lul
        options.Cookie.SecurePolicy = CookieSecurePolicy.Always;
        options.Cookie.SameSite = SameSiteMode.Strict;
        // options.Cookie.Name = "bloom_cookie";


        options.Cookie.Name = "bloom_cookie";
        options.Cookie.HttpOnly = true;
        options.Cookie.SecurePolicy = CookieSecurePolicy.Always;
    });


builder.Services.AddSession(options =>
{
    options.IdleTimeout = TimeSpan.FromMinutes(30);
    options.Cookie.HttpOnly = true;
    options.Cookie.IsEssential = true;
});

// Enable CORS for development
// TODO: add production check
builder.Services.AddCors(options => {
    options.AddDefaultPolicy(policy => {
        policy
            .AllowAnyOrigin()
            .AllowAnyHeader()
            .AllowAnyMethod();
    });
});

// authorization policies
builder.Services.AddAuthorization(options =>
{
    // options.AddPolicy(
    //     //only Admins can create accounts
    //     "CanCreateAccount", policy => policy.RequireRole("Admin", "Facilitator"));
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