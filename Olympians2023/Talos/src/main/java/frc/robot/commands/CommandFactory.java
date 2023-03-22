package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
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
/* 
    public Command getChaseTargetCommand() {
        return new ChaseTargetCommand(this.bot.getDriveTrain(), this.chaseForwardController, this.chaseAngularController);
    }
    */
    
    public Command getAutoBalanceCommand() {
        return new AutoBalanceCommand(this.bot.getDriveTrain());
    }

    public Command getElbowHoldAbsolutePositionCommand(double absolutePosition) {
        var arm = this.bot.getArm();
        var shoulder = arm.getShoulder();
        var elbow = arm.getElbow();
        return Commands.run(()->elbow.setPosition(absolutePosition-shoulder.getPosition()), elbow);
    }
/*
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

    public Command getDriveShoulderCommand(DoubleSupplier speedSupplier) {
        return Commands.run(() -> this.bot.getShoulder().driveShoulder(speedSupplier.getAsDouble()), bot.getShoulder());
    }

    public Command getDriveElbowCommand(DoubleSupplier speedSupplier){
        return Commands.run(() -> this.bot.getElbow().drive(speedSupplier.getAsDouble()), bot.getElbow());
    }

    public Command getDriveClawCommand(DoubleSupplier speedSuppler) {
        return Commands.run(() -> this.bot.getClaw().drive(speedSuppler.getAsDouble()), this.bot.getClaw());
    }
    */

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
