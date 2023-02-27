using Microsoft.Extensions.Options;
using System.Media;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();

builder.Services.Configure<SoundBoardSettings>(builder.Configuration.GetSection("sfx"));

var app = builder.Build();

//if (app.Environment.IsDevelopment())
//{
    app.UseSwagger();
    app.UseSwaggerUI();
//}

var playerMap = app.Services.GetRequiredService<IOptions<SoundBoardSettings>>().Value.Clips!
    .ToDictionary(
    clip => clip.FileName!, 
    clip => new SoundPlayer($"sfx/{clip.FileName}.wav"));

foreach (var player in playerMap.Values)
{
    player.LoadAsync();
}

app.MapGet("/list", (IOptions<SoundBoardSettings> boardSettings) => boardSettings.Value.Clips);
app.MapGet("/Play", (string fileName) =>
{
    if (playerMap.TryGetValue(fileName, out var player))
    {
        player.Play();
        return Results.Ok();
    }   
    else
    {
        return Results.NotFound(fileName);
    }
});

app.Run();

internal class Clip
{
    public string? Title { get; set; }
    public string? FileName { get; set; }
}

internal class SoundBoardSettings
{
    public IEnumerable<Clip>? Clips { get; set; }
}