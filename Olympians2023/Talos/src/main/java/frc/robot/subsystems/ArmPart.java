package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public abstract class ArmPart extends TunablePIDSubsystem {

    private final CANSparkMax motor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward feedforward;
    private final double positionOffset;

    private final DoublePublisher absPublisher;
    
    public ArmPart(
        String name,
            int motorCANId,
            int encoderPWMChannel,
            double positionOffset,
            double staticGain,
            double gravityGain,
            double velocityGain) {
        super("Arm/" + name);
        this.setName(name);
        this.positionOffset = positionOffset;
        this.motor = new CANSparkMax(motorCANId, MotorType.kBrushless);
        this.encoder = new DutyCycleEncoder(encoderPWMChannel);
        this.encoder.setDistancePerRotation(360);
        this.feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);

        var nt = NetworkTableInstance.getDefault();
        this.absPublisher = nt.getDoubleTopic("Arm/" + name + "/abs").publish();
    }

    public ArmFeedforward getFeedforward() {
        return this.feedforward;
    }

    public void drive(double speed) {
        this.motor.set(speed);
    }

    public DutyCycleEncoder getEncoder() {
        return this.encoder;
    }

    protected CANSparkMax getMotor() {
        return this.motor;
    }

    /**
     * Relative position in degrees offset from from the home position
     * @return
     */
    public double getPosition() {
        return this.encoder.getAbsolutePosition() - this.positionOffset;
    }

    public void setPosition(double position) {        
        this.setGoal(position);
        this.enable();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = this.feedforward.calculate(setpoint.position, setpoint.velocity);
        this.motor.setVoltage(output + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return this.getPosition();
    }

    @Override
    public void periodic() {
        super.periodic();
        this.absPublisher.accept(this.encoder.getAbsolutePosition());
    }
}
