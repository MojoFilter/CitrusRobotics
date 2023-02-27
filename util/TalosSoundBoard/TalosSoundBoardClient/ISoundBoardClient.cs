namespace TalosSoundBoardClient
{
    public interface ISoundBoardClient
    {
        Task PlayGameOver();
        Task PlayReady();
    }
}