using System.IO;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using bloom.Data;
using bloom.Services;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Moq;
using Xunit;

namespace bloom.Tests.Integration;

public class RsrAudioUploadTests : IClassFixture<BloomWebApplicationFactory>
{
    private readonly BloomWebApplicationFactory _factory;
    public RsrAudioUploadTests(BloomWebApplicationFactory factory) { _factory = factory; }

    [Theory]
    [InlineData("audio/webm;codecs=opus", "audio/webm; codecs=opus")]
    [InlineData("audio/ogg; codecs=opus", "audio/ogg; codecs=opus")]
    [InlineData("", "audio/webm")]
    public async Task ForwardsBrowserRecordingWithCodecParameters(string contentType, string expected)
    {
        using var scope = _factory.Services.CreateScope();
        var handler = new CaptureAudio(expected);
        using var client = new HttpClient(handler) { BaseAddress = new System.Uri("http://scorer") };
        var factory = new Mock<IHttpClientFactory>();
        factory.Setup(f => f.CreateClient("AutoRsr")).Returns(client);
        var service = new RsrService(factory.Object, scope.ServiceProvider.GetRequiredService<BloomDbContext>(), NullLogger<RsrService>.Instance);
        using var stream = new MemoryStream(new byte[] { 1, 2, 3 });
        var file = new FormFile(stream, 0, stream.Length, "sessionAudio", "session.webm")
        { Headers = new HeaderDictionary(), ContentType = contentType };
        var result = await service.AnalyzeAsync(file, 72, 5, "[]");
        Assert.True(handler.Called);
        Assert.Equal(2, result.TotalScore);
    }

    private sealed class CaptureAudio(string expected) : HttpMessageHandler
    {
        public bool Called { get; private set; }
        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            var parts = Assert.IsType<MultipartFormDataContent>(request.Content);
            var audio = parts.Single(p => p.Headers.ContentDisposition?.Name?.Trim('"') == "session_audio");
            Assert.Equal(expected, audio.Headers.ContentType?.ToString());
            Assert.Equal(new byte[] { 1, 2, 3 }, await audio.ReadAsByteArrayAsync(cancellationToken));
            Called = true;
            return new HttpResponseMessage(HttpStatusCode.OK) { Content = new StringContent(
                "{\"totalScore\":2,\"result\":\"test\",\"sentences\":[]}") };
        }
    }
}
