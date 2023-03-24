package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class ArcadeDriveCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final DoubleSupplier speedInput;
    private final DoubleSupplier rotationInput;
    private final String arcadeMode;
    private final String rumblePattern;

    public ArcadeDriveCommand(DoubleSupplier speedInput, DoubleSupplier rotationInput, String arcadeMode, String rumblePattern,
            DriveTrain driveTrain) {
        this.speedInput = speedInput;
        this.rotationInput = rotationInput;
        this.arcadeMode = arcadeMode;
        this.rumblePattern = rumblePattern;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer rob = RobotContainer.getInstance();
        rob.rumble(this.rumblePattern);
        if (this.arcadeMode.equalsIgnoreCase("split")) {
            rob.playArcadeSplit();
        } else {
            rob.playArcade1Stick();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var speed = this.speedInput.getAsDouble(); 
        final var rotation = this.rotationInput.getAsDouble();
        this.driveTrain.arcadeDrive(speed, rotation);
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
