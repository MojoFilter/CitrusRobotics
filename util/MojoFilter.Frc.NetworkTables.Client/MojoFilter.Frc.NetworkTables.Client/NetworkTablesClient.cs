using System.Collections.ObjectModel;
using System.Net.WebSockets;
using System.Reactive.Linq;
using System.Text.Json;
using System.Text;
using System.Reactive.Disposables;
using System.Reactive.Subjects;
using MessagePack;
using MojoFilter.Frc.NetworkTables.Client.Messages;
using Polly;
using System.Reactive;
using System.Diagnostics;

namespace MojoFilter.Frc.NetworkTables.Client;

public sealed class NetworkTablesClient : INetworkTablesClient
{
    public NetworkTablesClient()
    {
        Observable.FromEventPattern<NotifyCollectionChangedEventHandler, NotifyCollectionChangedEventArgs>(
            x => this.topics.CollectionChanged += x,
            x => this.topics.CollectionChanged -= x)
                 .Select(ev => ev.EventArgs);
    }

    public bool IsConnected => this.client?.State is WebSocketState.Open;

    public IObservable<NotifyCollectionChangedEventArgs> TopicsChanged { get; }

    public IEnumerable<string> Topics => this.topics.AsEnumerable();

    private static long connectId = 0;

    public IDisposable Connect(string host, string clientName, int port = 5810) {
        var id = connectId++;
        var disposable = new CancellationDisposable();
        var policy = Policy.Handle<Exception>()
                           .WaitAndRetryForeverAsync(_ => TimeSpan.FromSeconds(1), (ex, ts) => {
                               Debug.WriteLine(ex);
                               Debug.WriteLine(ex.Message);
                           });

        policy.ExecuteAsync(async (ct) => {
            Debug.WriteLine($"#{id} TRYING TO CONNECT TO {host}");
            this.client?.Dispose();
            this.client = BuildClient();
            var timeoutSource = new CancellationTokenSource(TimeSpan.FromSeconds(6));
            await Task.Run(
                ()=>this.client.ConnectAsync(new Uri($"ws://{host}:{port}/nt/{clientName}"), ct),
                timeoutSource.Token);
            this.connectionSubject.OnNext(true);
            await SubscribeAsync(KnownTopics.ServerPub, ct);
            await this.ReceiveMessagesAsync(ct);
        }, disposable.Token);

        return disposable;
    }

    public IObservable<string> SubscribeString(string topic) =>
        this.SubscribeTopic(topic, m => Encoding.UTF8.GetString(m.Span));

    public IObservable<T> SubscribeTopic<T>(string topic, Func<ReadOnlyMemory<byte>, T> deserialize) =>
        this.NewConnections.Select(_ =>
           Observable.Create<T>(obs => {
               CancellationTokenSource cts = new();
               Observable.FromAsync(() => this.SubscribeAsync(topic, cts.Token))
                         .Subscribe(_ => { }, obs.OnError, () => { }, cts.Token);

               var subscription =
                   this.topicDataSubject
                       .Where(gram => gram.Topic == topic)
                       .Select(gram => deserialize(gram.RawValue))
                       .Subscribe(obs);

               return new CompositeDisposable(cts, subscription);

           }))
        .Switch();

    private async Task ReceiveMessagesAsync(CancellationToken cancellationToken)
    {
        ArraySegment<byte> readBuffer = new(new byte[BufferSize]);
        while (!cancellationToken.IsCancellationRequested)
        {
            using MemoryStream ms = new();
            WebSocketReceiveResult result;
            do
            {
                result = await this.client.ReceiveAsync(readBuffer, cancellationToken);
                await ms.WriteAsync(readBuffer.Array!, readBuffer.Offset, result.Count, cancellationToken);
            } while (!result.EndOfMessage);

            ms.Seek(0, SeekOrigin.Begin);
            switch (result.MessageType)
            {
                case WebSocketMessageType.Text:
                    await ProcessTextMessageAsync(ms, cancellationToken);
                    break;
                case WebSocketMessageType.Binary:
                    await ProcessBinaryMessage(ms.ToArray(), cancellationToken);
                    break;
                default:
                    break;
            };
        }
    }

    private async Task SubscribeAsync(string topic, CancellationToken cancellationToken)
    {
        var msg = new[]
        {
            new TextDto(Methods.Subscribe, new
            {
                topics = new[] { topic },
                subuid = nextSubId++
            })
        };
        using MemoryStream ms = new();
        await JsonSerializer.SerializeAsync(ms, msg, cancellationToken: cancellationToken);
        var json = Encoding.UTF8.GetString(ms.ToArray());
        this.textMessagesSubject.OnNext($"Sent: {json}");
        await this.client.SendAsync(ms.ToArray(), WebSocketMessageType.Text, true, cancellationToken);
    }

    private async Task ProcessTextMessageAsync(Stream inputStream, CancellationToken cancellationToken)
    {
        var messages = await JsonSerializer.DeserializeAsync<IEnumerable<TextDto>>(inputStream, cancellationToken: cancellationToken);
        if (messages is not null)
        {
            foreach (var msg in messages)
            {
                var textMessage = msg switch
                {
                    { method: Methods.Announce } when msg.@params is JsonElement args => OnAnnounce(args),
                    _ => $"{msg.method}: {msg.@params}"
                };
                this.textMessagesSubject.OnNext(textMessage);
            }
        }
    }

    private ValueTask ProcessBinaryMessage(ReadOnlyMemory<byte> data, CancellationToken cancellationToken)
    {
        try
        {
            var frame = MessagePackSerializer.Deserialize<DataFrame>(data, cancellationToken: cancellationToken);
            if (frame.SourceId == serverPubId)
            {
                this.OnServerPub(frame.Data);
            }
            else if (this.topicIds.TryGetValue(frame.SourceId, out var topic))
            {
                this.topicDataSubject.OnNext(new(topic, frame.Data));
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine(ex.ToString());
        }

        return ValueTask.CompletedTask;
    }

    private string OnAnnounce(JsonElement announceArgs)
    {
        string? topic = announceArgs.GetProperty("name").GetString();
        uint id = announceArgs.GetProperty("id").GetUInt32();
        switch (topic)
        {
            case KnownTopics.ServerPub:
                this.serverPubId = id;
                break;
            default:
                this.topicIds[id] = $"{topic}";
                break;
        }
        return announceArgs.ToString();
    }

    private void OnServerPub(ReadOnlyMemory<byte> messagePack)
    {
        var json = MessagePackSerializer.ConvertToJson(messagePack);
        var pubs = JsonSerializer.Deserialize<IEnumerable<PublishDto>>(json);
        // this fails, but the extra json step shouldn't be neccessary
        //var pubs = MessagePackSerializer.Deserialize<PublishDto[]>(messagePack);
        foreach (var topic in pubs.Select(p => p.topic).OfType<string>())
        {
            this.topics.Add(topic);
        }
    }

    private IObservable<Unit> NewConnections => connectionSubject.Where(c => c).Select(_=>Unit.Default);


    private uint nextSubId = 0;
    private uint? serverPubId;
    private Task? receiveTask;
    private ClientWebSocket? client;

    private readonly ObservableCollection<string> topics = new ObservableCollection<string>();
    private readonly ISubject<TopicData> topicDataSubject = new Subject<TopicData>();
    private readonly ISubject<string> textMessagesSubject = new Subject<string>();
    private readonly ISubject<bool> connectionSubject = new BehaviorSubject<bool>(false);
    private readonly Dictionary<uint, string> topicIds = new Dictionary<uint, string>();

    private static ClientWebSocket BuildClient()
    {
        var client = new ClientWebSocket();
        client.Options.AddSubProtocol("networktables.first.wpi.edu");
        client.Options.SetBuffer(BufferSize, BufferSize);        
        return client;
    }

    private static readonly int BufferSize = 2 * 1024 * 1024;

    private record class TopicData(string Topic, ReadOnlyMemory<byte> RawValue);
    private record class TextDto(string method, object @params);

    private static class Methods
    {
        public const string Announce = "announce";
        public const string Subscribe = "subscribe";
    }

    private static class KnownTopics
    {
        public const string ServerPub = "$serverpub";
    }
}
