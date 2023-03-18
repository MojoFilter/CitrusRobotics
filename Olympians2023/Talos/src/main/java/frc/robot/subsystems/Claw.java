package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class Claw extends SubsystemBase {
    private final CANSparkMax motor;

    public Claw() {
        this.motor = new CANSparkMax(CanIds.ClawMotor, MotorType.kBrushless);
    }

    public void drive(double speed) {
        this.motor.set(speed * 0.5);
    }
}
