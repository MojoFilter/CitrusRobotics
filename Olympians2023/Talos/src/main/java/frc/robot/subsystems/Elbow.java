package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class Elbow extends SubsystemBase {
    
    private final CANSparkMax motor;

    public Elbow() {
        this.motor = new CANSparkMax(CanIds.ElbowMotor, MotorType.kBrushless);
    }

    public void drive(double speed) {
        this.motor.set(speed * .5);
    }
}
