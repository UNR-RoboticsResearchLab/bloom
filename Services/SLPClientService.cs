// bloom
// SLPClientService.cs
// Service managing SLPClient records (student-SLP associations).

using bloom.Data;
using bloom.Models;

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
    }
}
