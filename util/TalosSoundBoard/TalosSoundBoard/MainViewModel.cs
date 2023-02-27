using ReactiveUI;
using System.Diagnostics;
using System.Net.Http.Json;
using System.Reactive;
using System.Reactive.Threading.Tasks;
using System.Windows.Input;
using System.Reactive.Linq;

namespace TalosSoundBoard;

internal class MainViewModel : ReactiveObject
{
    public MainViewModel()
    {
        
        _clips = this.GetClips().ToObservable().ToProperty(this, nameof(Clips), scheduler: RxApp.MainThreadScheduler);
        this.PlayCommand = ReactiveCommand.CreateFromTask<string>(this.Play);
    }

    private async Task<Unit> Play(string fileName)
    {
        var client = new HttpClient();
        await client.GetAsync($"http://gordon:5851/play?fileName={fileName}");
        return Unit.Default;
    }

    public ReactiveCommand<string, Unit> PlayCommand { get; }
    public ICommand TestCommand { get; }

    public IEnumerable<Clip> Clips => _clips.Value;
    ObservableAsPropertyHelper<IEnumerable<Clip>> _clips;

    private async Task<IEnumerable<Clip>> GetClips()
    {
        var client = new HttpClient();
        return await client.GetFromJsonAsync<IEnumerable<Clip>>("http://gordon:5851/list");        
    }


}

record class Clip(string Title, string FileName);
