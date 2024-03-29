package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Dashboard;
import frc.robot.Constants.DriveSettings;

public class DashboardManager extends SubsystemBase {

    private GenericEntry speedGovernorSource;
    private SendableChooser<Command> driveModeChooser = new SendableChooser<>();
    private Command currentDriveCommand;

    public void configureDashboard(
            DriveTrain driveTrain,
            Arm arm,
            Command arcadeSplitCommand,
            Command arcade1StickCommand,
            Command tankDriveCommand,
            Command curvatureDriveCommand) {
        this.setupDriveTab(driveTrain, arcadeSplitCommand, arcade1StickCommand, tankDriveCommand,
                curvatureDriveCommand);
        this.setupTestTab(driveTrain, arm);
    }

    public double getSpeedGovernor() {
        return this.speedGovernorSource.getDouble(DriveSettings.DefaultSpeedGovernor);
    }

    @Override
    public void periodic() {
        this.updateDriveMode();
    }

    private void updateDriveMode() {
        var selectedCommand = this.driveModeChooser.getSelected();
        if (selectedCommand != this.currentDriveCommand) {
            this.currentDriveCommand = selectedCommand;
            this.currentDriveCommand.schedule();
        }
    }

    private void setupTestTab(DriveTrain driveTrain, Arm arm) {
        var testTab = Shuffleboard.getTab(Dashboard.TestTabName);

        testTab.add("Navx", driveTrain.getNav());
        testTab.add("Left Encoder", driveTrain.getLeftEncoder());
        testTab.add("Right Encoder", driveTrain.getRightEncoder());
        testTab.add("Shoulder", arm.getShoulder().getEncoder());
        testTab.add("Elbow", arm.getElbow());
    }

    private void setupDriveTab(
            DriveTrain driveTrain,
            Command arcadeSplitCommand,
            Command arcade1StickCommand,
            Command tankDriveCommand,
            Command curvatureDriveCommand) {
        ShuffleboardTab driveTab = Shuffleboard.getTab(Constants.Dashboard.DriveTabName);

        this.currentDriveCommand = arcadeSplitCommand;
        this.driveModeChooser.setDefaultOption(Constants.Dashboard.DriveModeOptions.ArcadeSplit, arcadeSplitCommand);
        this.driveModeChooser.addOption(Constants.Dashboard.DriveModeOptions.Arcade1Stick, arcade1StickCommand);
        this.driveModeChooser.addOption(Constants.Dashboard.DriveModeOptions.Tank, tankDriveCommand);
        this.driveModeChooser.addOption(Constants.Dashboard.DriveModeOptions.Curvature, curvatureDriveCommand);
        driveTab.add(Constants.Dashboard.DriveModeTitle, this.driveModeChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        this.speedGovernorSource = driveTab.add(Constants.Dashboard.SpeedGovernorTitle, 0.65)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withSize(4, 1)
                .withPosition(2, 0)
                .getEntry();

        driveTab.add(driveTrain.getNav())
                .withSize(2, 2)
                .withPosition(6, 0);

        driveTab.add(driveTrain.getField())
                .withSize(6, 4)
                .withPosition(0, 1);

        driveTab.add(driveTrain.getDrive())
                .withPosition(6, 2);

        driveTab.addCamera(
                Constants.Dashboard.DriveCameraTitle,
                Constants.Dashboard.DriveCameraName,
                Constants.Dashboard.DriveCameraUrls).withPosition(8, 0);

    }
}
