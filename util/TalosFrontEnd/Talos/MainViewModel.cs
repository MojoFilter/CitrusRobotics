namespace Talos;

public interface IMainViewModel {
    string Host { get; set; }
    ISettings Settings { get; }
    IObservable<string> Connect();
}

internal class MainViewModel : ReactiveObject, IMainViewModel 
{

    public MainViewModel(INetworkTablesClient client, ISettings settings)
    {
        this.client = client;
        this.Settings = settings;
        _host = settings.DefaultHost;
    }

    public ISettings Settings { get; }

    public string Host {
        get => _host;
        set => this.RaiseAndSetIfChanged(ref _host, value);
    }

    public IObservable<string> Connect() {
        this.connection?.Dispose();
        this.connection = this.client.Connect(this.Host, "Talos");
        return this.client
                   .SubscribeString("talos/sfx/clip")
                   .Where(fileName => !string.IsNullOrWhiteSpace(fileName))
                   .Do(file => Debug.WriteLine($"Play file: {file}"))
                   .ObserveOn(RxApp.MainThreadScheduler);
    }

    private readonly INetworkTablesClient client;
    private IDisposable connection;
    private string _host;
}
