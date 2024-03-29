package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class Claw extends SubsystemBase {

    private final CANSparkMax motor;
    private final SlewRateLimiter limiter;

    public Claw() {
        super();
        this.motor = new CANSparkMax(CanIds.ClawMotor, MotorType.kBrushless);
        this.limiter = new SlewRateLimiter(1.0);
    }

    public void setSpeed(double closingSpeed) {
        closingSpeed *= 0.2;
        var limitedRate = this.limiter.calculate(closingSpeed);
        this.motor.set(limitedRate);
    }
}
