namespace MojoFilter.Frc.NetworkTables.Client;

public interface INetworkTablesClient
{
    bool IsConnected { get; }
    IEnumerable<string> Topics { get; }
    IObservable<NotifyCollectionChangedEventArgs> TopicsChanged { get; }

    IDisposable Connect(string host, string clientName, int port = 5810);

    IObservable<string> SubscribeString(string topic);

    //IPublisher<string> CreateStringPublisher(string topic);
}

public interface IPublisher<T> : IDisposable {
    Task WriteAsync(T value, CancellationToken cancellationToken);
}