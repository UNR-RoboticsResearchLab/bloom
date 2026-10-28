using System;
using System.IO;
using System.Net.WebSockets;
using System.Threading.Tasks;
using bloom.Services.RemoteShell;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Moq;
using Xunit;

namespace bloom.Tests.Services
{
    /// <summary>
    /// ShellRelay logic that needs a controllable clock. Everything that needs real sockets is in
    /// Integration/RemoteShellRelayTests.
    /// </summary>
    public class ShellRelayTests
    {
        private sealed class ManualTime : TimeProvider
        {
            public DateTimeOffset Now { get; set; } = new(2026, 1, 1, 12, 0, 0, TimeSpan.Zero);
            public override DateTimeOffset GetUtcNow() => Now;
        }

        private static ShellRelay CreateRelay(ManualTime time, int opensPerMinute = 3) =>
            new(Mock.Of<IServiceScopeFactory>(),
                Options.Create(new RemoteShellOptions { SessionOpensPerMinute = opensPerMinute }),
                time,
                NullLogger<ShellRelay>.Instance);

        private static OpenRequest OpenOfflineRobot(string adminId) =>
            new(Guid.NewGuid(), "robot", adminId, null,
                new ShellSocket(WebSocket.CreateFromStream(new MemoryStream(), new WebSocketCreationOptions { IsServer = true })),
                80, 24, Force: false);

        [Fact]
        public async Task RateLimit_AppliesPerAdmin_AndSlidesAfterAMinute()
        {
            var time = new ManualTime();
            var relay = CreateRelay(time);

            for (var i = 0; i < 3; i++)
                Assert.Equal(OpenStatus.Offline, (await relay.OpenSessionAsync(OpenOfflineRobot("alice"))).Status);
            Assert.Equal(OpenStatus.RateLimited, (await relay.OpenSessionAsync(OpenOfflineRobot("alice"))).Status);

            // Another admin has their own budget.
            Assert.Equal(OpenStatus.Offline, (await relay.OpenSessionAsync(OpenOfflineRobot("bob"))).Status);

            time.Now += TimeSpan.FromSeconds(59);
            Assert.Equal(OpenStatus.RateLimited, (await relay.OpenSessionAsync(OpenOfflineRobot("alice"))).Status);

            time.Now += TimeSpan.FromSeconds(1);
            Assert.Equal(OpenStatus.Offline, (await relay.OpenSessionAsync(OpenOfflineRobot("alice"))).Status);
        }
    }
}
