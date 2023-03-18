package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class Shoulder extends SubsystemBase {
    
    private final CANSparkMax shoulderMotor;

    public Shoulder() {
        this.shoulderMotor = new CANSparkMax(CanIds.ShoulderMotor, MotorType.kBrushless);
    }

    public void driveShoulder(double speed) {
        this.shoulderMotor.set(speed * 0.25);
    }
}
