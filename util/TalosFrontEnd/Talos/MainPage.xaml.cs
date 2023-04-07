namespace Talos {
    public partial class MainPage : ContentPage
    {
        public MainPage(IMainViewModel viewModel)
        {
            this.viewModel = viewModel;
            this.BindingContext = viewModel;
            InitializeComponent();
        }

        protected override void OnAppearing() {
            DeviceDisplay.Current.KeepScreenOn = true;
            this.Connect();
        }


        private void PlayFile(string fileName) {
            try {
                this.player.Source = MediaSource.FromResource(fileName);
                this.player.Play();
            } catch (Exception ex) {
                // oh well
            }
        }

        private readonly IMainViewModel viewModel;
        private readonly SerialDisposable subscriptionContainer = new SerialDisposable();

        private async void SettingsClicked(object sender, EventArgs e) {
            var popup = new SettingsPopup(
                this.viewModel.Host,
               this.viewModel.Settings,
               this.viewModel.Locales,
               this.viewModel.Locale);
            var result = await this.ShowPopupAsync(popup);
            if (result is SettingsPopup.Settings settings) {
                this.viewModel.Host = settings.Host;
                this.viewModel.Locale = settings.locale;
                //this.Connect();
            }
        }

        private void Connect() {
            this.subscriptionContainer.Disposable = this.viewModel.Connect().Subscribe(this.PlayFile);
        }

        //private async void Button_Clicked(object sender, EventArgs e) {
        //    await this.viewModel.Say(this.ttsEntry.Text);
        //}
    }
}