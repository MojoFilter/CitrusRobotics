package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class TankDrive extends CommandBase {

    private final DriveTrain m_driveTrain;
    private DoubleSupplier m_left;
    private DoubleSupplier m_right;

    public TankDrive(DoubleSupplier left, DoubleSupplier right, DriveTrain subsystem) {
        m_left = left;
        m_right = right;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);
    }

    public TankDrive(double left, double right, DriveTrain driveTrain) {
        this(() -> left, () -> right, driveTrain);
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
        m_driveTrain.drive(m_left.getAsDouble(), m_right.getAsDouble());
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
