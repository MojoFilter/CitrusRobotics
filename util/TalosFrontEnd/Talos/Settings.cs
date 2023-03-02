namespace Talos;

public interface ISettings {
    IReadOnlyList<string> Hosts { get; }
    string DefaultHost { get; }
}

internal class Settings : ISettings {
    public IReadOnlyList<string> Hosts { get; } = new[] {
        "roborio-9300-frc",
        "10.93.00.02",
        "clio"
    };

    public string DefaultHost => Hosts.FirstOrDefault();
}
