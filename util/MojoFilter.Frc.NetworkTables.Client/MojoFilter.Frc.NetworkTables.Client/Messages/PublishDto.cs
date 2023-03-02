namespace MojoFilter.Frc.NetworkTables.Client.Messages;

[MessagePackObject]
public class PublishDto
{
    [Key(0)]
    public int uid { get; set; }

    [Key(1)]
    public string? topic { get; set; }
}
