// bloom
// SLPClientService.cs
// Service managing SLPClient records (student-SLP associations).

using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class SLPClientService : ISLPClientService
    {
        private readonly BloomDbContext _context;

        public SLPClientService(BloomDbContext context)
        {
            _context = context;
        }

        public async Task<SLPClient> CreateAsync(string name, string slpId, string studentId)
        {
            var slp = await _context.Accounts.FindAsync(slpId)
                ?? throw new KeyNotFoundException($"SLP account {slpId} not found");

            var client = new SLPClient
            {
                Name = name,
                StudentId = studentId,
                CreatedDate = DateTime.UtcNow,
                Teachers = new List<Account> { slp }
            };

            _context.SLPClients.Add(client);
            await _context.SaveChangesAsync();

            return client;
        }

        public async Task<SLPClientResponseDto?> CreateClientAsync(string name, string slpId, string studentId)
        {
            var student = await _context.Accounts.FindAsync(studentId);
            if (student == null) return null;

            var slp = await _context.Accounts.FindAsync(slpId);
            if (slp == null) return null;

            var client = new SLPClient
            {
                Name = name,
                StudentId = studentId,
                CreatedDate = DateTime.UtcNow,
                Teachers = new List<Account> { slp }
            };

            _context.SLPClients.Add(client);
            await _context.SaveChangesAsync();

            return new SLPClientResponseDto
            {
                Id = client.Id,
                Name = client.Name,
                StudentId = client.StudentId,
                StudentName = student.FullName
            };
        }

        public async Task<List<SLPClientResponseDto>> GetClientsForSlpAsync(string slpId)
        {
            return await _context.SLPClients
                .Where(c => c.Teachers!.Any(t => t.Id == slpId))
                .Select(c => new SLPClientResponseDto
                {
                    Id = c.Id,
                    Name = c.Name,
                    StudentId = c.StudentId,
                    StudentName = c.Student!.FullName
                })
                .ToListAsync();
        }

        public async Task<SLPClientResponseDto?> GetClientAsync(Guid id)
        {
            var client = await _context.SLPClients
                .Include(c => c.Student)
                .Include(c => c.Assignments!)
                    .ThenInclude(a => a.Lesson)
                .FirstOrDefaultAsync(c => c.Id == id);

            if (client == null) return null;

            var progress = await _context.LessonProgresses
                .Include(p => p.Lesson)
                .Where(p => p.StudentId == client.StudentId)
                .ToListAsync();

            return new SLPClientResponseDto
            {
                Id = client.Id,
                Name = client.Name,
                StudentId = client.StudentId,
                StudentName = client.Student?.FullName,
                Assignments = (client.Assignments ?? []).Select(a => new AssignmentResponseDto
                {
                    Id = a.Id,
                    StudentId = a.StudentId,
                    LessonId = a.LessonId,
                    LessonTitle = a.Lesson?.Title ?? string.Empty,
                    AssignedById = a.AssignedById,
                    AssignedDate = a.AssignedDate,
                    DueDate = a.DueDate,
                    IsCompleted = a.IsCompleted
                }).ToList(),
                Progress = progress.Select(p => new StudentLessonProgressDto
                {
                    Id = p.Id,
                    LessonId = p.LessonId,
                    LessonTitle = p.Lesson?.Title ?? string.Empty,
                    LessonStep = p.LessonStep,
                    ProgressPercentage = p.ProgressPercentage,
                    LastUpdated = p.LastUpdated
                }).ToList()
            };
        }

        public async Task<AddTeacherResult> AddTeacherAsync(Guid clientId, string teacherId)
        {
            var client = await _context.SLPClients
                .Include(c => c.Teachers)
                .FirstOrDefaultAsync(c => c.Id == clientId);

            if (client == null) return AddTeacherResult.ClientNotFound;

            var teacher = await _context.Accounts.FindAsync(teacherId);
            if (teacher == null) return AddTeacherResult.TeacherNotFound;

            client.Teachers ??= new List<Account>();

            if (client.Teachers.Any(t => t.Id == teacherId))
                return AddTeacherResult.AlreadyAssociated;

            client.Teachers.Add(teacher);
            await _context.SaveChangesAsync();

            return AddTeacherResult.Added;
        }
    }
}
