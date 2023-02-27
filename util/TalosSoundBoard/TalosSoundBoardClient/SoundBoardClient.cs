using System.Net;
using System.Net.Sockets;

namespace TalosSoundBoardClient;

public class SoundBoardClient : ISoundBoardClient
{
    public Task PlayGameOver() => PlayIndex(0);
    public Task PlayReady() => PlayIndex(1);

    private async Task PlayIndex(byte index)
    {
        var client = new UdpClient();
        client.EnableBroadcast = true;
        await client.SendAsync(new[] { index }, 1, "255.255.255.255", 11000);
    }
}