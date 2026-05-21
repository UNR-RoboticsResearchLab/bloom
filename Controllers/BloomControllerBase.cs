using System.Security.Claims;
using Microsoft.AspNetCore.Mvc;

namespace bloom.Controllers
{
    /// <summary>
    /// Base class for all Bloom API controllers. Provides shared helpers for the authenticated request context.
    /// </summary>
    public abstract class BloomControllerBase : ControllerBase
    {
        /// <summary>
        /// Returns the identity ID of the currently authenticated user, or null if unauthenticated.
        /// </summary>
        protected string? GetCurrentUserId() =>
            User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
    }
}
