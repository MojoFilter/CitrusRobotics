package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmSettings;
import frc.robot.Constants.RumblePatterns;
import frc.robot.RobotContainer;

public final class CommandFactory {

    private final RobotContainer bot;

    public CommandFactory(RobotContainer bot) {
        this.bot = bot;
    }

    public Command getArcadeDriveCommand() {
        return this.getArcadeDriveCommand(4, "Split", RumblePatterns.ArcadeSplit);
    }

    public Command getStickDriveCommand() {
        return this.getArcadeDriveCommand(0, "1-Stick", RumblePatterns.Arcade1Stick);
    }

    public Command getTankDriveCommand() {
        return new TankDriveCommand(
                () -> -bot.getXboxController1().getRawAxis(1) * bot.getSpeedGovernor(),
                () -> -bot.getXboxController1().getRawAxis(5) * bot.getSpeedGovernor(),
                bot.getDriveTrain())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command getCurvatureDriveCommand() {
        return Commands.run(() -> this.bot.getDriveTrain().curvatureDrive(
            -bot.getXboxController1().getRawAxis(1) * bot.getSpeedGovernor(),
            -bot.getXboxController1().getRawAxis(4) * bot.getSpeedGovernor()))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command getTwistCommand() {
        return new TwistCommand(this.bot.getDriveTrain()).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command getChaseTargetCommand() {
        return new ChaseTargetCommand(this.bot.getDriveTrain());
    }

    public Command getAutoBalanceCommand() {
        return new AutoBalanceCommand(this.bot.getDriveTrain());
    }

    public Command getArmTrackSetpointCommand() {
        var arm = this.bot.getArm();    
        var defaultPosition = ArmSettings.Shoulder.DefaultPositionDegrees;
        Preferences.initDouble(ArmSettings.Shoulder.PositionKey, defaultPosition);
        return Commands.runOnce(()-> {
            var setPoint = Preferences.getDouble(ArmSettings.Shoulder.PositionKey, defaultPosition);
            arm.setGoal(setPoint);
            arm.enable();
        }, arm);
    }

    private Command getArcadeDriveCommand(int rotationAxis, String arcadeMode,
            String rumblePattern) {
        DoubleSupplier speedInput = () -> -bot.getXboxController1().getRawAxis(1) * bot.getSpeedGovernor();
        DoubleSupplier rotationInput = () -> -bot.getXboxController1().getRawAxis(rotationAxis)
                * bot.getSpeedGovernor();
        return new ArcadeDriveCommand(speedInput, rotationInput, arcadeMode,
                rumblePattern,
                bot.getDriveTrain())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}
