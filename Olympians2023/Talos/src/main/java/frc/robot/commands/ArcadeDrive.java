package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


/**
 *
 */
public class ArcadeDrive extends CommandBase {

        private final DriveTrain m_driveTrain;
    private DoubleSupplier m_speed;
    private DoubleSupplier m_rotation;
    private String m_arcadeMode;
    private String m_rumblePattern;
 

    public ArcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, String arcadeMode, String rumblePattern, DriveTrain subsystem) {
        m_speed = speed;
        m_rotation = rotation;
        m_arcadeMode = arcadeMode;
        m_rumblePattern = rumblePattern;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer rob = RobotContainer.getInstance();
        rob.rumble(m_rumblePattern);
        if (m_arcadeMode.equalsIgnoreCase("split")) {
            rob.playArcadeSplit();
        }
        else {
            rob.playArcade1Stick();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(m_speed.getAsDouble(), m_rotation.getAsDouble());
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
