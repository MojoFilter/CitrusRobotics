package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Balancer;

public class AutoBalanceCommand extends CommandBase {
    
    private final DriveTrain driveTrain;
    private final Balancer balancer;

    public AutoBalanceCommand(DriveTrain driveTrain) {
        this.addRequirements(driveTrain);
        this.driveTrain = driveTrain;
        this.balancer = new Balancer(driveTrain.getAccelerometer());
    }

    @Override
    public void initialize() {
        this.balancer.reset();
    }

    @Override
    public void execute() {
        var speed = this.balancer.getBalanceSpeed();
        this.driveTrain.tankDrive(speed, speed);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
