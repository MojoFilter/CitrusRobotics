namespace TalosSoundBoard;

public partial class MainPage : ContentPage
{
	public MainPage()
	{
		this.BindingContext = new MainViewModel();
		InitializeComponent();
	}

}

