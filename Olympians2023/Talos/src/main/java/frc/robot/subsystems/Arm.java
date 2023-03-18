package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmSettings;
import frc.robot.Constants.CanIds;

public class Arm extends ProfiledPIDSubsystem {
    private final Encoder shoulderEncoder;
    private final DCMotor shoulderGearbox;
    private final CANSparkMax shoulderMotor;
    private final ArmFeedforward shoulderFeedforward;

    // sim
    private final SingleJointedArmSim shoulderSim;
    private final EncoderSim shoulderEncoderSim;

    private final Mechanism2d mech2d;
    private final MechanismRoot2d shoulderMech;
    private final MechanismLigament2d upperArmMech;
    private final MechanismLigament2d foreArmMech;

    public Arm() {
        super(
                new ProfiledPIDController(
                        ArmSettings.Shoulder.DefaultP,
                        ArmSettings.Shoulder.DefaultI,
                        ArmSettings.Shoulder.DefaultD,
                        new TrapezoidProfile.Constraints(
                                ArmSettings.Shoulder.MaxVelocityRadsPerSecond,
                                ArmSettings.Shoulder.MaxAccelerationMetersPerSecondSquared)),
                0);
        this.shoulderEncoder = new Encoder(ArmSettings.Shoulder.EncoderAChannel, ArmSettings.Shoulder.EncoderBChannel);
        this.shoulderEncoder.setDistancePerPulse(ArmSettings.Shoulder.EncoderDistancePerPulse);
        this.shoulderGearbox = DCMotor.getAndymark9015(1);
        this.shoulderMotor = new CANSparkMax(CanIds.ShoulderMotor, MotorType.kBrushless);

        this.shoulderFeedforward = new ArmFeedforward(
                ArmSettings.Shoulder.SVolts,
                ArmSettings.Shoulder.GVolts,
                ArmSettings.Shoulder.VVoltSecondPerRad,
                ArmSettings.Shoulder.AVoltSecondSquaredPerRad);

        this.shoulderSim = new SingleJointedArmSim(
                this.shoulderGearbox,
                ArmSettings.Shoulder.GearReduction,
                SingleJointedArmSim.estimateMOI(ArmSettings.UpperArmLengthMeters, ArmSettings.UpperArmMassKgs),
                ArmSettings.UpperArmLengthMeters,
                ArmSettings.Shoulder.MinAngleRads,
                ArmSettings.Shoulder.MaxAngleRads,
                true,
                VecBuilder.fill(ArmSettings.Shoulder.EncoderDistancePerPulse));

        this.shoulderEncoderSim = new EncoderSim(this.shoulderEncoder);

        this.mech2d = new Mechanism2d(60, 60);
        this.shoulderMech = this.mech2d.getRoot("Shoulder", 30, 30);
        this.upperArmMech = this.shoulderMech.append(new MechanismLigament2d("UpperArm", 30, -90));
        this.upperArmMech.setColor(new Color8Bit(Color.kBlue));
        this.foreArmMech = this.shoulderMech.append(
                new MechanismLigament2d(
                        "Forearm",
                        30,
                        Units.radiansToDegrees(this.shoulderSim.getAngleRads()),
                        6,
                        new Color8Bit(Color.kYellow)));

        Preferences.initDouble(ArmSettings.Shoulder.PositionKey, ArmSettings.Shoulder.DefaultPositionDegrees);
        Preferences.initDouble(ArmSettings.Shoulder.PKey, ArmSettings.Shoulder.DefaultP);

    }

    @Override
    public void simulationPeriodic() {
        var shoulderInput = this.shoulderMotor.get() * RobotController.getBatteryVoltage();
        this.shoulderSim.setInput(shoulderInput);
        this.shoulderSim.update(0.020);

        this.shoulderEncoderSim.setDistance(this.shoulderSim.getAngleRads());
        var voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(this.shoulderSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(voltage);

        this.foreArmMech.setAngle(Units.radiansToDegrees(this.shoulderSim.getAngleRads()));
    }

    public Mechanism2d getMechanism() {
        return this.mech2d;
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        var feedForward = this.shoulderFeedforward.calculate(setpoint.position, setpoint.velocity);
        this.shoulderMotor.setVoltage(output + feedForward);
    }

    @Override
    protected double getMeasurement() {
        return this.shoulderEncoder.getDecodingScaleFactor();
    }
}
