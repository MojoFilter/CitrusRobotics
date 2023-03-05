
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;


/**
 *
 */
public class Twist extends SequentialCommandGroup {

    private static final double TWIST_SPEED = 0.75;
    private static final double TWIST_DURATION = 0.25; // seconds
    private static final double PAUSE_DURATION = 0.25; // seconds

    public Twist(DriveTrain driveTrain){
        addCommands(
                spinLeft(TWIST_DURATION / 2.0, driveTrain),
                Commands.waitSeconds(PAUSE_DURATION),
                spinRight(TWIST_DURATION, driveTrain),
                Commands.waitSeconds(PAUSE_DURATION),
                spinLeft(TWIST_DURATION, driveTrain),
                Commands.waitSeconds(PAUSE_DURATION),
                spinRight(TWIST_DURATION / 2.0, driveTrain));
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private Command spinLeft(double seconds, DriveTrain drivetrain) {
        return this.spin(-TWIST_SPEED, TWIST_SPEED, seconds, drivetrain);
    }

    private Command spinRight(double seconds, DriveTrain drivetrain) {
        return this.spin(TWIST_SPEED, -TWIST_SPEED, seconds, drivetrain);
    }

    private Command spin(double leftSpeed, double rightSpeed, double seconds, DriveTrain driveTrain) {
        return new TankDrive(leftSpeed, rightSpeed, driveTrain).withTimeout(seconds);
    }
}
