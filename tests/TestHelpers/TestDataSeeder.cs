using System;
using System.Linq;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using Microsoft.AspNetCore.Identity;

namespace bloom.Tests.TestHelpers
{
    /// <summary>
    /// Builds the minimum valid EF graph needed to exercise lesson-runtime / robot-session
    /// flows without going through a full HTTP signup/login for every test. Every call mints
    /// fresh Guids/emails, so tests stay independent even though integration test classes share
    /// one SQLite connection (and thus one database) for the lifetime of a test class (see
    /// BloomWebApplicationFactory). Call these fresh per test method -- do not seed once in a
    /// class fixture constructor and reuse the result across [Fact]s.
    /// </summary>
    public static class TestDataSeeder
    {
        /// <summary>
        /// Password used for every account created via CreateAccountAsync. AuthTestHelper logs
        /// in with this same constant, since integration tests authenticate via a real
        /// POST /api/User/login call rather than minting a token directly.
        /// </summary>
        public const string DefaultPassword = "Test@12345";

        /// <summary>
        /// Creates an Account through UserManager, so normalized email/username, password hash,
        /// and security stamp are set exactly like in production. Use this for integration tests
        /// that authenticate as the account (see AuthTestHelper).
        /// </summary>
        public static async Task<Account> CreateAccountAsync(
            UserManager<Account> userManager, string role = "SLP", string? emailPrefix = null)
        {
            var suffix = Guid.NewGuid().ToString("N")[..8];
            var email = $"{emailPrefix ?? role.ToLowerInvariant()}-{suffix}@test.bloom";

            var account = new Account
            {
                UserName = email,
                Email = email,
                FullName = $"{role} Test {suffix}",
                CreatedDate = DateTime.UtcNow,
                Role = role,
                EmailConfirmed = true
            };

            var result = await userManager.CreateAsync(account, DefaultPassword);
            if (!result.Succeeded)
            {
                throw new InvalidOperationException(
                    "Failed to seed test account: " +
                    string.Join(", ", result.Errors.Select(e => e.Description)));
            }

            return account;
        }

        /// <summary>
        /// Creates an Account by writing directly to the DbContext, skipping Identity entirely.
        /// Cheaper for pure service-layer unit tests that never touch UserManager --
        /// RobotSessionService only ever treats UserId as an opaque FK string.
        /// </summary>
        public static Account CreateRawAccount(BloomDbContext db, string role = "SLP")
        {
            var suffix = Guid.NewGuid().ToString("N")[..8];
            var account = new Account
            {
                UserName = $"{role.ToLowerInvariant()}-{suffix}@test.bloom",
                Email = $"{role.ToLowerInvariant()}-{suffix}@test.bloom",
                FullName = $"{role} Test {suffix}",
                CreatedDate = DateTime.UtcNow,
                Role = role
            };
            db.Accounts.Add(account);
            db.SaveChanges();
            return account;
        }

        public static Robot CreateRobot(BloomDbContext db, string? name = null)
        {
            var robot = new Robot
            {
                Name = name ?? $"Test Robot {Guid.NewGuid():N}",
                ManufactureDate = DateTime.UtcNow
            };
            db.Robots.Add(robot);
            db.SaveChanges();
            return robot;
        }

        /// <summary>
        /// Builds a Lesson with `totalSteps` LessonSteps (StepOrder 1..N). LessonStep.Id is
        /// mapped ValueGeneratedNever() in BloomDbContext, so every step must get an explicit
        /// Guid here -- leaving it default would make EF treat the step as an existing row
        /// being updated rather than a new one being inserted.
        /// </summary>
        public static Lesson CreateLessonWithSteps(BloomDbContext db, string createdById, int totalSteps = 3)
        {
            var lesson = new Lesson
            {
                Title = $"Test Lesson {Guid.NewGuid():N}",
                CreatedById = createdById,
                CreatedDate = DateTime.UtcNow,
                TotalSteps = totalSteps,
                LessonType = LessonType.Speech
            };

            for (var i = 1; i <= totalSteps; i++)
            {
                lesson.Steps.Add(new LessonStep
                {
                    Id = Guid.NewGuid(),
                    LessonId = lesson.Id,
                    StepOrder = i,
                    Type = "Speech",
                    Script = $"Step {i} script"
                });
            }

            db.Lessons.Add(lesson);
            db.SaveChanges();
            return lesson;
        }

        public static async Task<RobotSession> CreateSessionAsync(
            BloomDbContext db, Robot robot, string? userId = null, string? sessionCode = null)
        {
            var session = new RobotSession
            {
                UserId = userId,
                SessionCode = sessionCode ?? RandomSessionCode(),
                CreatedAt = DateTime.UtcNow,
                LastUpdatedAt = DateTime.UtcNow,
                Robots = 1
            };
            db.RobotSessions.Add(session);
            await db.SaveChangesAsync();
            return session;
        }

        public static Assignment CreateAssignment(
            BloomDbContext db, string studentId, Guid lessonId, string assignedById)
        {
            var assignment = new Assignment
            {
                StudentId = studentId,
                LessonId = lessonId,
                AssignedById = assignedById,
                AssignedDate = DateTime.UtcNow
            };
            db.Assignments.Add(assignment);
            db.SaveChanges();
            return assignment;
        }

        /// <summary>
        /// One-shot convenience covering the common case for lesson-runtime integration tests:
        /// an SLP account, a Robot, a Lesson with `totalSteps` steps (created by the SLP), and a
        /// RobotSession owned by the SLP with that robot already "joined" (Robots = 1).
        /// </summary>
        public static async Task<(Account Slp, Robot Robot, Lesson Lesson, RobotSession Session)>
            SeedLessonRuntimeGraphAsync(BloomDbContext db, UserManager<Account> userManager, int totalSteps = 3)
        {
            var slp = await CreateAccountAsync(userManager, role: "SLP");
            var robot = CreateRobot(db);
            var lesson = CreateLessonWithSteps(db, slp.Id, totalSteps);
            var session = await CreateSessionAsync(db, robot, userId: slp.Id);
            return (slp, robot, lesson, session);
        }

        private static string RandomSessionCode() => Random.Shared.Next(100000, 999999).ToString();
    }
}
