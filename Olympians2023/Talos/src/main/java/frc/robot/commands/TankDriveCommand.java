package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class TankDriveCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rightInput;
    private final SlewRateLimiter leftLimiter;
    private final SlewRateLimiter rightLimiter;

    public TankDriveCommand(double constantLeftInput, double constantRightInput, DriveTrain driveTrain) {
        this(()->constantLeftInput, ()->constantRightInput, driveTrain);
    }
    
    public TankDriveCommand(DoubleSupplier leftInput, DoubleSupplier rightInput, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.leftInput = leftInput;
        this.rightInput = rightInput;
        this.addRequirements(driveTrain);
        this.leftLimiter = new SlewRateLimiter(Constants.DriveTrain.SpeedRateLimit);
        this.rightLimiter = new SlewRateLimiter(Constants.DriveTrain.SpeedRateLimit);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer rob = RobotContainer.getInstance();
        rob.rumble(Constants.RumblePatterns.TankDrive);
        rob.playTankDrive();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var leftSpeed = this.leftLimiter.calculate(this.leftInput.getAsDouble());
        var rightSpeed = this.rightLimiter.calculate(this.rightInput.getAsDouble());
        this.driveTrain.tankDrive(leftSpeed, rightSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
