// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.*;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private PWMVictorSPX left1;
private PWMVictorSPX left2;
private MotorControllerGroup leftController;
private PWMVictorSPX right1;
private PWMVictorSPX right2;
private MotorControllerGroup rightController;
private DifferentialDrive drive;
private Encoder leftEncoder;
private Encoder rightEncoder;
private AnalogGyro gyro;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private AnalogGyroSim gyroSim;
    private DifferentialDrivetrainSim driveSim;
    private Field2d field;
    private DifferentialDriveOdometry odometry;

    /**
    *
    */
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
left1 = new PWMVictorSPX(0);
 addChild("Left 1",left1);
 left1.setInverted(false);

left2 = new PWMVictorSPX(1);
 addChild("Left 2",left2);
 left2.setInverted(false);

leftController = new MotorControllerGroup(left1, left2  );
 addChild("Left Controller",leftController);
 

right1 = new PWMVictorSPX(2);
 addChild("Right 1",right1);
 right1.setInverted(false);

right2 = new PWMVictorSPX(3);
 addChild("Right 2",right2);
 right2.setInverted(false);

rightController = new MotorControllerGroup(right1, right2  );
 addChild("Right Controller",rightController);
 

drive = new DifferentialDrive(leftController, rightController);
 addChild("Drive",drive);
 drive.setSafetyEnabled(true);
drive.setExpiration(0.1);
drive.setMaxOutput(1.0);


leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
 addChild("Left Encoder",leftEncoder);
 leftEncoder.setDistancePerPulse(1.0);

rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
 addChild("Right Encoder",rightEncoder);
 rightEncoder.setDistancePerPulse(1.0);

gyro = new AnalogGyro(0);
 addChild("Gyro",gyro);
 gyro.setSensitivity(0.007);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        this.leftEncoderSim = new EncoderSim(this.leftEncoder);
        this.rightEncoderSim = new EncoderSim(this.rightEncoder);
        this.gyroSim = new AnalogGyroSim(gyro);
        this.driveSim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k10p71,
            KitbotWheelSize.kSixInch,
            null);

        this.leftEncoder.setDistancePerPulse(Constants.DriveTrain.EncoderDistancePerPulse); // not sure what these constants should be
        this.rightEncoder.setDistancePerPulse(Constants.DriveTrain.EncoderDistancePerPulse);
        this.resetEncoders();
        this.field = new Field2d();
        SmartDashboard.putData("Field", field);
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
        SmartDashboard.putString(kDriveMode, "");
    }

    private final String kDriveMode = "Drive Mode";

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        
        
        driveSim.setInputs(leftController.get() * RobotController.getInputVoltage(),
         rightController.get() * RobotController.getInputVoltage());  
        
         driveSim.update(0.02);

        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-driveSim.getHeading().getDegrees());
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        SmartDashboard.putString(kDriveMode, "Arcade Drive");
        drive.arcadeDrive(speed, rotation);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
    }

    public void resetEncoders() {
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }
}

