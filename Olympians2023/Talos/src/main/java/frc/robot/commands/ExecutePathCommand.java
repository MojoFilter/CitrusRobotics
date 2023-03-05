package frc.robot.commands;

import com.pathplanner.lib.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.*;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class ExecutePathCommand extends SequentialCommandGroup {

    public ExecutePathCommand(String pathName, DriveTrain driveTrain) {
        addCommands(
                this.buildAutoPath(driveTrain, pathName));
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private Command buildAutoPath(DriveTrain drive, String pathName) {
        Trajectory trajectory = PathPlanner.loadPath(pathName, new PathConstraints(
                Constants.Auto.MaxSpeedMetersPerSecond, Constants.Auto.MaxAccelerationMetersPerSecondSquared));

        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                drive::getPose,
                new RamseteController(Constants.Auto.RamseteB, Constants.Auto.RamseteZ),
                new SimpleMotorFeedforward(
                        Constants.DriveTrain.StaticGainVolts,
                        Constants.DriveTrain.VelocityGainVolts,
                        Constants.DriveTrain.AccelerationGainVolts),
                Constants.DriveTrain.DriveKinematics,
                drive::getWheelSpeeds,
                new PIDController(Constants.DriveTrain.PDriveVelocity, 0, 0),
                new PIDController(Constants.DriveTrain.PDriveVelocity, 0, 0),
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts,
                drive);

        // Reset odometry to the starting pose of the trajectory.
        drive.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }
}
