using System.Media;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Xml.XPath;

const int SoundBoardPort = 11000;
var listener = new UdpClient(SoundBoardPort);
var endpoint = new IPEndPoint(IPAddress.Any, SoundBoardPort);
var player = new SoundPlayer("gameover.wav");
try
{
    player.Load();
    Console.WriteLine("Waiting for command");
    //var result = await listener.ReceiveAsync();
    //Console.WriteLine($"Received '{Encoding.UTF8.GetString(result.Buffer)}' so playing whatever");
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