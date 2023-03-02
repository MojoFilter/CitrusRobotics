namespace MojoFilter.Frc.NetworkTables.Client.Messages;

[MessagePackObject]
public class DataFrame
{
    [Key(0)]
    public uint SourceId { get; set; }

    [Key(1)]
    public ulong Timestamp { get; set; }

    [Key(2)]
    public uint DataType { get; set; }

    [Key(3)]
    public ReadOnlyMemory<byte> Data { get; set; }
}
