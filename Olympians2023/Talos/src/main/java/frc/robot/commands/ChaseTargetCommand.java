package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.DriveTrain;

public class ChaseTargetCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final PhotonCamera tagetCam;
    private final DoubleEntry targetRangeEntry;
    private final PIDController forwardController;
    private final PIDController turnController;

    public ChaseTargetCommand(DriveTrain driveTrain, PIDController forwardController, PIDController angularController) {
        this.driveTrain = driveTrain;
        this.tagetCam = new PhotonCamera(Vision.TargetCameraName);
        this.addRequirements(driveTrain);

        var nt = NetworkTableInstance.getDefault();
        var topic = nt.getDoubleTopic("Drive/TargetRange");
        this.targetRangeEntry = topic.getEntry(-1.0);

        this.forwardController = forwardController;
        this.turnController = angularController;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var forwardSpeed = 0.0;
        var rotationSpeed = 0.0;
        var range = -1.0;

        var result = this.tagetCam.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    Vision.TargetCameraOffsetMetersY,
                    Vision.ChaseTargetHeightMeters,
                    Vision.TargetCameraPitch,
                    Units.degreesToRadians(target.getPitch()));
            forwardSpeed = -this.forwardController.calculate(Math.abs(range), -Vision.ChaseTargetRangeMeters);
            rotationSpeed = -this.turnController.calculate(target.getYaw(), 0.0);
        }
        this.driveTrain.arcadeDrive(forwardSpeed, rotationSpeed);
        this.targetRangeEntry.set(range);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
