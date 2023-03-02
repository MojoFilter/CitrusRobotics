namespace Talos;

public partial class SettingsPopup : Popup
{
	public SettingsPopup(string host, ISettings settings)
	{
		InitializeComponent();
		this.ResultWhenUserTapsOutsideOfPopup = host;
		this.hostPicker.ItemsSource = settings.Hosts.ToList();
		this.hostPicker.SelectedItem = host;
	}

    private void Button_Clicked(object sender, EventArgs e) {
		this.Close(this.hostPicker.SelectedItem as string);
    }
}