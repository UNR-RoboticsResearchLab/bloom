// bloom
// SLPClientService.cs
// SLP Client service business logic layer
// Created: 02/21/2026

using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class SLPClientService : ISLPClientService
    {
        private readonly BloomDbContext _dbContext;
        private readonly IAccountService _accountService;

        public SLPClientService(BloomDbContext context, IAccountService accountService)
        {
            _dbContext = context;
            _accountService = accountService;
        }

        public async Task<bool> CreateAsync(string name, string teacherId, Guid studentId)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(name) || name.Length < 3)
                {
                    throw new ArgumentException("SLP Client name must be at least 3 characters long");
                }

                var teacher = await _accountService.GetByIdAsync(teacherId);
                if (teacher == null)
                {
                    throw new KeyNotFoundException($"Teacher with id {teacherId} not found");
                }

                var student = await _dbContext.Accounts.FindAsync(studentId);
                if (student == null)
                {
                    throw new KeyNotFoundException($"Student with id {studentId} not found");
                }

                var slpClient = new SLPClient
                {
                    Name = name,
                    StudentId = studentId,
                    CreatedDate = DateTime.UtcNow
                };

                if (slpClient.Teachers == null)
                {
                    slpClient.Teachers = new List<Account>();
                }
                slpClient.Teachers.Add(teacher);

                await _dbContext.SLPClients.AddAsync(slpClient);
                await _dbContext.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error creating SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task DeleteByIdAsync(Guid id)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients.FindAsync(id);
                if (slpClient != null)
                {
                    _dbContext.SLPClients.Remove(slpClient);
                    await _dbContext.SaveChangesAsync();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error deleting SLP Client: {ex.Message}");
            }
        }

        public async Task<SLPClient?> GetByIdAsync(Guid id)
        {
            try
            {
                return await _dbContext.SLPClients
                    .Include(c => c.Student)
                    .Include(c => c.Teachers)
                    .Include(c => c.Assignments)
                    .Include(c => c.Lessons)
                    .FirstOrDefaultAsync(c => c.Id == id);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting SLP Client by id: {ex.Message}");
                return null;
            }
        }

        public async Task<IEnumerable<SLPClient>> GetAllAsync()
        {
            try
            {
                return await _dbContext.SLPClients
                    .Include(c => c.Student)
                    .Include(c => c.Teachers)
                    .ToListAsync();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting all SLP Clients: {ex.Message}");
                return [];
            }
        }

        public async Task<IEnumerable<SLPClient>> GetByTeacherIdAsync(string teacherId)
        {
            try
            {
                if (string.IsNullOrEmpty(teacherId))
                {
                    throw new ArgumentNullException(nameof(teacherId));
                }

                return await _dbContext.SLPClients
                    .Where(c => c.Teachers!.Any(t => t.Id == teacherId))
                    .Include(c => c.Student)
                    .Include(c => c.Teachers)
                    .ToListAsync();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting SLP Clients by teacher id: {ex.Message}");
                return [];
            }
        }

        public async Task<IEnumerable<SLPClient>> GetByStudentIdAsync(Guid studentId)
        {
            try
            {
                return await _dbContext.SLPClients
                    .Where(c => c.StudentId == studentId)
                    .Include(c => c.Student)
                    .Include(c => c.Teachers)
                    .ToListAsync();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error getting SLP Clients by student id: {ex.Message}");
                return [];
            }
        }

        public async Task<bool> ModifyAsync(SLPClient slpClient)
        {
            try
            {
                ArgumentNullException.ThrowIfNull(slpClient);

                var existingSLPClient = await _dbContext.SLPClients.FindAsync(slpClient.Id);
                if (existingSLPClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClient.Id} not found");
                }

                existingSLPClient.Name = slpClient.Name;
                existingSLPClient.AccentColor = slpClient.AccentColor;
                existingSLPClient.BackgroundImageUrl = slpClient.BackgroundImageUrl;
                existingSLPClient.UpdatedDate = DateTime.UtcNow;

                _dbContext.SLPClients.Update(existingSLPClient);
                await _dbContext.SaveChangesAsync();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error modifying SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> AddTeacherAsync(Guid slpClientId, string teacherId)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients
                    .Include(c => c.Teachers)
                    .FirstOrDefaultAsync(c => c.Id == slpClientId);

                if (slpClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClientId} not found");
                }

                var teacher = await _accountService.GetByIdAsync(teacherId);
                if (teacher == null)
                {
                    throw new KeyNotFoundException($"Teacher with id {teacherId} not found");
                }

                if (slpClient.Teachers == null)
                {
                    slpClient.Teachers = new List<Account>();
                }

                if (!slpClient.Teachers.Any(t => t.Id == teacherId))
                {
                    slpClient.Teachers.Add(teacher);
                    _dbContext.SLPClients.Update(slpClient);
                    await _dbContext.SaveChangesAsync();
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error adding teacher to SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> RemoveTeacherAsync(Guid slpClientId, string teacherId)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients
                    .Include(c => c.Teachers)
                    .FirstOrDefaultAsync(c => c.Id == slpClientId);

                if (slpClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClientId} not found");
                }

                if (slpClient.Teachers != null)
                {
                    var teacher = slpClient.Teachers.FirstOrDefault(t => t.Id == teacherId);
                    if (teacher != null)
                    {
                        slpClient.Teachers.Remove(teacher);
                        _dbContext.SLPClients.Update(slpClient);
                        await _dbContext.SaveChangesAsync();
                    }
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error removing teacher from SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> AddLessonAsync(Guid slpClientId, Guid lessonId)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients
                    .Include(c => c.Lessons)
                    .FirstOrDefaultAsync(c => c.Id == slpClientId);

                if (slpClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClientId} not found");
                }

                var lesson = await _dbContext.Lessons.FindAsync(lessonId);
                if (lesson == null)
                {
                    throw new KeyNotFoundException($"Lesson with id {lessonId} not found");
                }

                if (slpClient.Lessons == null)
                {
                    slpClient.Lessons = new List<Lesson>();
                }

                if (!slpClient.Lessons.Any(l => l.Id == lessonId))
                {
                    slpClient.Lessons.Add(lesson);
                    _dbContext.SLPClients.Update(slpClient);
                    await _dbContext.SaveChangesAsync();
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error adding lesson to SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> RemoveLessonAsync(Guid slpClientId, Guid lessonId)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients
                    .Include(c => c.Lessons)
                    .FirstOrDefaultAsync(c => c.Id == slpClientId);

                if (slpClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClientId} not found");
                }

                if (slpClient.Lessons != null)
                {
                    var lesson = slpClient.Lessons.FirstOrDefault(l => l.Id == lessonId);
                    if (lesson != null)
                    {
                        slpClient.Lessons.Remove(lesson);
                        _dbContext.SLPClients.Update(slpClient);
                        await _dbContext.SaveChangesAsync();
                    }
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error removing lesson from SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> AddAssignmentAsync(Guid slpClientId, Guid assignmentId)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients
                    .Include(c => c.Assignments)
                    .FirstOrDefaultAsync(c => c.Id == slpClientId);

                if (slpClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClientId} not found");
                }

                var assignment = await _dbContext.Assignments.FindAsync(assignmentId);
                if (assignment == null)
                {
                    throw new KeyNotFoundException($"Assignment with id {assignmentId} not found");
                }

                if (slpClient.Assignments == null)
                {
                    slpClient.Assignments = new List<Assignment>();
                }

                if (!slpClient.Assignments.Any(a => a.Id == assignmentId))
                {
                    slpClient.Assignments.Add(assignment);
                    _dbContext.SLPClients.Update(slpClient);
                    await _dbContext.SaveChangesAsync();
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error adding assignment to SLP Client: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> RemoveAssignmentAsync(Guid slpClientId, Guid assignmentId)
        {
            try
            {
                var slpClient = await _dbContext.SLPClients
                    .Include(c => c.Assignments)
                    .FirstOrDefaultAsync(c => c.Id == slpClientId);

                if (slpClient == null)
                {
                    throw new KeyNotFoundException($"SLP Client with id {slpClientId} not found");
                }

                if (slpClient.Assignments != null)
                {
                    var assignment = slpClient.Assignments.FirstOrDefault(a => a.Id == assignmentId);
                    if (assignment != null)
                    {
                        slpClient.Assignments.Remove(assignment);
                        _dbContext.SLPClients.Update(slpClient);
                        await _dbContext.SaveChangesAsync();
                    }
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error removing assignment from SLP Client: {ex.Message}");
                return false;
            }
        }
    }
}
