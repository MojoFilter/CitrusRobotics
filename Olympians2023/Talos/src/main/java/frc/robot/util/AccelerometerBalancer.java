package frc.robot.util;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.Constants.Balance;

public class AccelerometerBalancer extends Balancer {
    private final Accelerometer accel;

    public AccelerometerBalancer(Accelerometer accel) {
        this.accel = accel;
        this.reset();
    }

    @Override
    protected double getPitch() {
        var ac = this.accel;
        return Math.atan2((-ac.getX()),
                Math.sqrt(ac.getY() * ac.getY() + ac.getZ() * ac.getZ())) * Balance.RAD2DEG;
    }

    @Override
    protected double getRoll() {
        return Math.atan2(this.accel.getY(), this.accel.getZ()) * Balance.RAD2DEG;
    }

}
