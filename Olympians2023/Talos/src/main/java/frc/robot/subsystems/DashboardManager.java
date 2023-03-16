package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DashboardManager extends SubsystemBase {

    private GenericEntry speedGovernorSource;
    private SendableChooser<Command> driveModeChooser = new SendableChooser<>();
    private Command currentDriveCommand;
    private PIDController chaseForwardController;
    private PIDController chaseTurnController;

    public void configureDashboard(
            DriveTrain driveTrain,
            PIDController chaseForwardController,
            PIDController chaseTurnController,
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

        driveTab.add(driveTrain.getGyro())
                .withSize(2, 2)
                .withPosition(6, 0);

        driveTab.add(driveTrain.getField())
                .withSize(6, 4)
                .withPosition(0, 1);

        driveTab.add(driveTrain.getDrive())
                .withPosition(6, 2);

        driveTab.add(driveTrain.getAccelerometer())
                .withPosition(6, 4);
                
/*         driveTab.addCamera(
            Constants.Dashboard.TargetCameraTitle,
            Constants.Dashboard.TargetCameraName,
            Constants.Dashboard.TargetCameraUrls); */
            
        driveTab.addCamera(
            Constants.Dashboard.DriveCameraTitle,
            Constants.Dashboard.DriveCameraName,
            Constants.Dashboard.DriveCameraUrls).withPosition(8, 0); 
        //Shuffleboard.selectTab(Constants.Dashboard.DriveTabName);

        driveTab.add("Chase Forward", chaseForwardController).withPosition(8, 2);
        driveTab.add("Chase Turn", chaseTurnController).withPosition(8, 3);
    }

    public double getSpeedGovernor() {
        return this.speedGovernorSource.getDouble(Constants.DriveTrain.DefaultSpeedGovernor);
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
}
