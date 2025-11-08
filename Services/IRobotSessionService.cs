
using bloom.Models;

namespace bloom.Services
{
    public interface IRobotSessionService
    {
        RobotSession StartSession(bool anon = false);
        
    }
}