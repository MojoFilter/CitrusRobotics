using System.ComponentModel.DataAnnotations;
using System.Media;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Xml.XPath;

const int SoundBoardPort = 11000;
var clipMap = new Dictionary<int, SoundPlayer>()
{
    {0, new SoundPlayer("gameover.wav") },
    {1, new SoundPlayer("ready_to_roll.wav") }
};

foreach (var clip in clipMap.Values)
{
    clip.Load();
}

var listener = new UdpClient(SoundBoardPort);
var endpoint = new IPEndPoint(IPAddress.Any, SoundBoardPort);
var player = new SoundPlayer("gameover.wav");
try
{
    player.Load();
    Console.WriteLine("Waiting for command");
    var result = await listener.ReceiveAsync();
    var data = result.Buffer.First();
    var cmd = data & 0xC0;
    switch (cmd) {
        default:
            Play(data & 0x3f);
            break;
    };
    player.Play();
    Console.ReadKey();
}
catch (Exception ex)
{
    Console.WriteLine(ex.Message);
}
finally
{
    listener.Close();
}

void Play(int clipIndex) {
    if (clipMap.TryGetValue(clipIndex, out var clip))
    {
        clip.Play();
    }
}