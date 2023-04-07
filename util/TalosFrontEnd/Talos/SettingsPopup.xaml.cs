namespace Talos;

public partial class SettingsPopup : Popup
{
	public SettingsPopup(string host, ISettings settings, IEnumerable<Locale> locales, Locale locale)
	{
		InitializeComponent();
		this.ResultWhenUserTapsOutsideOfPopup = host;
		this.hostPicker.ItemsSource = settings.Hosts.ToList();
		this.hostPicker.SelectedItem = host;
		this.localePicker.ItemsSource = locales.ToList();
		this.localePicker.SelectedItem = locale;

	}

    private void Button_Clicked(object sender, EventArgs e) {
		var result = new Settings(this.hostPicker.SelectedItem as string, this.localePicker.SelectedItem as Locale);
		this.Close(result);
    }

	public record class Settings(string Host, Locale locale);
}