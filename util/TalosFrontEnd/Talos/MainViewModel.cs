using System.Reactive;
using System.Reactive.Threading.Tasks;

namespace Talos;

public interface IMainViewModel {
    string Host { get; set; }
    ISettings Settings { get; }
    Locale Locale { get; set; }
    IEnumerable<Locale> Locales { get; }
    float Pitch { get; set; }
    float Volume { get; set; }

    IObservable<string> Connect();
    Task<Unit> Say(string phrase);
}

internal class MainViewModel : ReactiveObject, IMainViewModel 
{

    public MainViewModel(INetworkTablesClient client, ISettings settings)
    {
        this.client = client;
        this.Settings = settings;
        _host = settings.DefaultHost;
        this.LoadLocales();
    }

    public ISettings Settings { get; }

    public IEnumerable<Locale> Locales {
        get => _locales;
        set => this.RaiseAndSetIfChanged(ref _locales, value);
    }

    public string Host {
        get => _host;
        set => this.RaiseAndSetIfChanged(ref _host, value);
    }

    public float Pitch {
        get => _pitch;
        set => this.RaiseAndSetIfChanged(ref _pitch, value);
    }

    public float Volume {
        get => _volume;
        set => this.RaiseAndSetIfChanged(ref _volume, value);
    }

    public Locale? Locale {
        get => _locale;
        set => this.RaiseAndSetIfChanged(ref _locale, value);
    }

    public IObservable<string> Connect() {
        this.connection?.Dispose();
        var connection = new CompositeDisposable {
            this.client.Connect(this.Host, "Talos"),
            this.client.SubscribeString("talos/sfx/tts")
                       .SelectMany(Say)
                       .Subscribe()
        };
        this.connection = connection;
        
        return this.client
                   .SubscribeString("talos/sfx/clip")
                   .Where(fileName => !string.IsNullOrWhiteSpace(fileName))
                   .Do(file => Debug.WriteLine($"Play file: {file}"))
                   .ObserveOn(RxApp.MainThreadScheduler);
    }

    public async Task<Unit> Say(string phrase) {
        SpeechOptions speechOptions = new() {
            Locale = this.Locale,
            Pitch = this.Pitch,
            Volume = this.Volume
        };
        await TextToSpeech.Default.SpeakAsync(phrase, speechOptions);
        return Unit.Default;
    }

    private async void LoadLocales() {
        this.Locales = (await TextToSpeech.GetLocalesAsync()).ToList();
        this.Locale = this.Locales.FirstOrDefault();
    }

    private readonly INetworkTablesClient client;
    private IDisposable connection;
    private string _host;
    private float _pitch = 1.0f;
    private float _volume = 1.0f;
    private IEnumerable<Locale> _locales;
    private Locale? _locale;
}
