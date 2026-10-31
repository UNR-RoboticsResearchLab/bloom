// bloom
// PublicUrl.cs
// Public origin used in links sent by email.

namespace bloom.Services.Email
{
    public static class PublicUrl
    {
        // App:PublicBaseUrl wins so links are right behind a proxy that does not forward the host;
        // otherwise use the request (which UseForwardedHeaders has already corrected).
        public static string GetBaseUrl(IConfiguration configuration, HttpContext? httpContext)
        {
            var configured = configuration["App:PublicBaseUrl"];
            if (!string.IsNullOrWhiteSpace(configured))
                return configured.TrimEnd('/');

            var request = httpContext?.Request
                ?? throw new InvalidOperationException(
                    "No request available to build the link. Set App:PublicBaseUrl.");
            return $"{request.Scheme}://{request.Host}{request.PathBase}";
        }
    }
}
