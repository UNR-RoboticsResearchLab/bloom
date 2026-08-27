using System.Text.Json;

namespace bloom.Tests.Integration
{
    /// <summary>
    /// Shared JsonSerializerOptions for integration tests. ASP.NET Core's MVC JSON
    /// formatter serializes response bodies with camelCase property names by default, but
    /// System.Net.Http.Json's ReadFromJsonAsync/PostAsJsonAsync use plain System.Text.Json
    /// defaults (case-sensitive) unless told otherwise. JsonSerializerDefaults.Web matches
    /// what ASP.NET Core itself uses (camelCase + case-insensitive matching), so response
    /// DTOs deserialize correctly regardless of casing.
    /// </summary>
    internal static class TestJson
    {
        public static readonly JsonSerializerOptions Options = new(JsonSerializerDefaults.Web);
    }
}
