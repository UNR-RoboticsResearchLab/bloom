using System;
using System.IO;
using System.Security.Claims;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using bloom.Controllers;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Moq;
using Xunit;

namespace bloom.Tests.Controllers;

public class RsrStudyControllerTests : IDisposable
{
    private readonly string _directory = Path.Combine(Path.GetTempPath(), "bloom-study-tests-" + Guid.NewGuid());
    private RsrStudyController Controller(string? owner = "examiner-1")
    {
        var config = new ConfigurationBuilder().AddInMemoryCollection(new[] {
            new System.Collections.Generic.KeyValuePair<string, string?>("RsrStudy:ArchivePath", _directory)
        }).Build();
        return new RsrStudyController(config, Mock.Of<IWebHostEnvironment>(), NullLogger<RsrStudyController>.Instance)
        {
            ControllerContext = new ControllerContext { HttpContext = new DefaultHttpContext {
                User = new ClaimsPrincipal(new ClaimsIdentity(owner == null ? Array.Empty<Claim>() : new[] {
                    new Claim(ClaimTypes.NameIdentifier, owner), new Claim(ClaimTypes.Role, "admin")
                }, "test"))
            } }
        };
    }
    private static IFormFile Audio() => new FormFile(new MemoryStream(Encoding.UTF8.GetBytes("full recording")), 0, 14, "audio", "session.webm");
    private static string Metadata(Guid id, string phase = "finished") => JsonSerializer.Serialize(new {
        id, phase, events = new[] { new { type = "recording_started", ms = 0 } }, attempts = Array.Empty<object>()
    });

    [Fact]
    public async Task ArchiveRevisionsAreAppendOnlyAndPrivateToTheirOwner()
    {
        var controller = Controller(); var id = Guid.NewGuid();
        var first = Assert.IsType<OkObjectResult>(await controller.Save(id, Metadata(id), Audio(), CancellationToken.None));
        var second = Assert.IsType<OkObjectResult>(await controller.Save(id, Metadata(id), Audio(), CancellationToken.None));
        var revision1 = JsonSerializer.SerializeToElement(first.Value).GetProperty("revision").GetGuid();
        var revision2 = JsonSerializer.SerializeToElement(second.Value).GetProperty("revision").GetGuid();
        Assert.NotEqual(revision1, revision2);
        Assert.IsType<PhysicalFileResult>(controller.Audio(id, revision1));
        Assert.IsType<PhysicalFileResult>(controller.Audio(id, revision2));
        Assert.IsType<NotFoundResult>(Controller("examiner-2").Audio(id, revision1));
        var list = Assert.IsType<OkObjectResult>(await Controller("examiner-2").List(CancellationToken.None));
        Assert.Equal(0, JsonSerializer.SerializeToElement(list.Value).GetArrayLength());
    }
    [Fact]
    public async Task RejectsLiveRecordingsAndMismatchedIdentifiers()
    {
        var controller = Controller(); var id = Guid.NewGuid();
        Assert.IsType<BadRequestObjectResult>(await controller.Save(id, Metadata(id, "recording"), Audio(), CancellationToken.None));
        Assert.IsType<BadRequestObjectResult>(await controller.Save(id, Metadata(Guid.NewGuid()), Audio(), CancellationToken.None));
        Assert.IsType<BadRequestObjectResult>(await controller.Save(id, "[]", Audio(), CancellationToken.None));
        Assert.False(Directory.Exists(_directory));
    }
    [Fact]
    public async Task RequiresAnIdentifiedOwner()
    {
        var controller = Controller(null); var id = Guid.NewGuid();
        Assert.IsType<UnauthorizedResult>(await controller.Save(id, Metadata(id), Audio(), CancellationToken.None));
        Assert.IsType<UnauthorizedResult>(await controller.List(CancellationToken.None));
        Assert.IsType<UnauthorizedResult>(controller.Audio(id, Guid.NewGuid()));
    }
    public void Dispose() { if (Directory.Exists(_directory)) Directory.Delete(_directory, true); }
}
