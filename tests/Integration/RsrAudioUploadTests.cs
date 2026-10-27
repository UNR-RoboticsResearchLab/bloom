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
    [InlineData("audio/webm;codecs=opus", "audio/webm; codecs=opus", false)]
    [InlineData("audio/webm;codecs=opus", "audio/webm; codecs=opus", true)]
    [InlineData("audio/ogg; codecs=opus", "audio/ogg; codecs=opus", false)]
    [InlineData("", "audio/webm", false)]
    public async Task ForwardsBrowserRecordingWithCodecParameters(string contentType, string expected, bool unscored)
    {
        using var scope = _factory.Services.CreateScope();
        var handler = new CaptureAudio(expected, unscored);
        using var client = new HttpClient(handler) { BaseAddress = new System.Uri("http://scorer") };
        var factory = new Mock<IHttpClientFactory>();
        factory.Setup(f => f.CreateClient("AutoRsr")).Returns(client);
        var service = new RsrService(factory.Object, scope.ServiceProvider.GetRequiredService<BloomDbContext>(), NullLogger<RsrService>.Instance);
        using var stream = new MemoryStream(new byte[] { 1, 2, 3 });
        var file = new FormFile(stream, 0, stream.Length, "sessionAudio", "session.webm")
        { Headers = new HeaderDictionary(), ContentType = contentType };
        var result = await service.AnalyzeAsync(file, 72, 5, "[]");
        Assert.True(handler.Called);
        Assert.Equal(unscored ? 0 : 2, result.TotalScore);
        if (unscored)
        {
            Assert.Null(result.Sentences[0].Score);
            Assert.NotNull(result.Sentences[0].AnalysisError);
            var saved = await service.GetAssessmentByIdAsync(result.AssessmentId);
            Assert.Null(saved!.Sentences[0].Score);
            Assert.NotNull(saved.Sentences[0].AnalysisError);
        }
    }

    private sealed class CaptureAudio(string expected, bool unscored) : HttpMessageHandler
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
                unscored ? "{\"totalScore\":0,\"result\":\"Incomplete\",\"sentences\":[{\"id\":\"1\",\"groundTruth\":\"test\",\"response\":\"response\",\"score\":null,\"errors\":null,\"analysisError\":\"Review manually\"}]}" :
                "{\"totalScore\":2,\"result\":\"test\",\"sentences\":[]}") };
        }
    }
}
