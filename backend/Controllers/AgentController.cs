// bloom
// AgentController.cs
// Anonymous endpoints for the robot agent that must work before it has credentials.

using System.Reflection;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    [ApiController]
    [Route("api/[controller]")]
    public class AgentController : BloomControllerBase
    {
        private static readonly string BackendVersion =
            typeof(AgentController).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion
            ?? "unknown";

        /// <summary>
        /// Fixed-shape response used by `robot-agent doctor` to detect captive portals and
        /// TLS-intercepting proxies: anything else coming back (a redirect, a login page) means
        /// something between the robot and the backend is answering instead.
        /// </summary>
        [HttpGet("health")]
        public IActionResult Health() => Ok(new { Service = "bloom", Status = "ok", Version = BackendVersion });
    }
}
