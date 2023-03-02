using CommunityToolkit.Maui;
using Microsoft.Extensions.Logging;
using MojoFilter.Frc.NetworkTables.Client;

namespace Talos
{
    public static class MauiProgram
    {
        public static MauiApp CreateMauiApp()
        {
            var builder = MauiApp.CreateBuilder();
            builder.Services.AddSingleton<IMainViewModel, MainViewModel>()
                            .AddTransient<INetworkTablesClient, NetworkTablesClient>()
                            .AddSingleton<ISettings, Settings>()
                            .AddSingleton<MainPage>();
            builder
                .UseMauiApp<App>()
                .UseMauiCommunityToolkit()
                .UseMauiCommunityToolkitMediaElement()
                .ConfigureFonts(fonts =>
                {
                    fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
                    fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
                });

#if DEBUG
		builder.Logging.AddDebug();
#endif

            return builder.Build();
        }
    }
}