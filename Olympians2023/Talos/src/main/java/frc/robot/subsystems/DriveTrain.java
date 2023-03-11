package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 *
 */
public class DriveTrain extends SubsystemBase {

    private PWMVictorSPX left1;
    private PWMVictorSPX left2;
    private MotorControllerGroup leftController;
    private PWMVictorSPX right1;
    private PWMVictorSPX right2;
    private MotorControllerGroup rightController;
    private DifferentialDrive drive;
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private final ADXRS450_Gyro gyro;
    private final Accelerometer accelerometer;
    private final PhotonCamera targetCam;

    private DifferentialDriveOdometry odometry;
    private final SimpleMotorFeedforward feedforward;
    private final DifferentialDrivePoseEstimator poseEstimator;

    private final PIDController leftPIDController;
    private final PIDController rightPIDController;

    private Field2d field;
    private Field2d fieldApproximation;

    private final AprilTagFieldLayout fieldTags;

    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private final ADXRS450_GyroSim gyroSim;
    private DifferentialDrivetrainSim driveSim;
    private final SimVisionSystem visionSim;

    private final IntegerEntry currentTargetEntry;

    private final UsbCamera driveCamera;
    private final MjpegServer camServer;

    /**
    *
    */
    public DriveTrain() {
        left1 = new PWMVictorSPX(0);
        addChild("Left 1", left1);
        left1.setInverted(false);

        left2 = new PWMVictorSPX(1);
        addChild("Left 2", left2);
        left2.setInverted(false);

        leftController = new MotorControllerGroup(left1, left2);
        addChild("Left Controller", leftController);

        right1 = new PWMVictorSPX(2);
        addChild("Right 1", right1);
        right1.setInverted(false);

        right2 = new PWMVictorSPX(3);
        addChild("Right 2", right2);
        right2.setInverted(false);

        rightController = new MotorControllerGroup(right1, right2);
        addChild("Right Controller", rightController);

        drive = new DifferentialDrive(leftController, rightController);
        addChild("Drive", drive);
        drive.setSafetyEnabled(true);
        drive.setExpiration(0.1);
        drive.setMaxOutput(1.0);

        leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        addChild("Left Encoder", leftEncoder);
        leftEncoder.setDistancePerPulse(1.0);

        rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        addChild("Right Encoder", rightEncoder);
        rightEncoder.setDistancePerPulse(1.0);

        this.gyro = new ADXRS450_Gyro();
        this.addChild("Gyro", this.gyro);

        this.accelerometer = new BuiltInAccelerometer();

        this.leftEncoderSim = new EncoderSim(this.leftEncoder);
        this.rightEncoderSim = new EncoderSim(this.rightEncoder);
        this.gyroSim = new ADXRS450_GyroSim(gyro);
        this.driveSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDualCIMPerSide,
                KitbotGearing.k10p71,
                KitbotWheelSize.kSixInch,
                null);
        this.visionSim = new SimVisionSystem(
                Constants.Vision.TargetCameraName,
                Constants.Vision.TargetCameraFOVDegrees,
                Constants.Vision.RobotToTargetCamera,
                Constants.Vision.TargetCameraMaxLedRangeMeters,
                Constants.Vision.TargetCameraResolutionWidth,
                Constants.Vision.TargetCameraResolutionHeight,
                Constants.Vision.TargetCameraMinTargetArea);

        this.leftEncoder.setDistancePerPulse(Constants.DriveTrain.EncoderDistancePerPulse); // not sure what these
                                                                                            // constants should be
        this.rightEncoder.setDistancePerPulse(Constants.DriveTrain.EncoderDistancePerPulse);
        this.resetEncoders();
        this.field = new Field2d();
        this.fieldApproximation = new Field2d();

        this.odometry = new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(getHeading()),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());

        this.poseEstimator = new DifferentialDrivePoseEstimator(
                Constants.DriveTrain.DriveKinematics,
                this.gyro.getRotation2d(),
                this.leftEncoder.getDistance(),
                this.rightEncoder.getDistance(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        this.feedforward = new SimpleMotorFeedforward(1, 3);
        this.leftPIDController = new PIDController(1, 0, 0);
        this.rightPIDController = new PIDController(1, 0, 0);

        try {
            this.fieldTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            /*
            List<AprilTag> testTags = new ArrayList<AprilTag>();
            testTags.add(new AprilTag(6, new Pose3d(8, 3, 1, new Rotation3d())));
            var length = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
            var width = Units.feetToMeters(26) + Units.inchesToMeters(3.5);
            this.fieldTags = new AprilTagFieldLayout(testTags, length, width);
            */
            this.visionSim.addVisionTargets(this.fieldTags);
        } catch (Exception ioe) {//IOException ioe) {
            ioe.printStackTrace();
            throw new RuntimeException();
        }

        this.targetCam = new PhotonCamera(Constants.Vision.TargetCameraName);

        var currentTargetTopic = NetworkTableInstance.getDefault().getIntegerTopic("Drive/CurrentTarget");
        this.currentTargetEntry = currentTargetTopic.getEntry(-1);

        this.camServer = new MjpegServer("Drive Camera Server", 1181);
        this.driveCamera = new UsbCamera("Drive Camera", 0);
        this.camServer.setSource(driveCamera);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        this.updateOdometry();
        this.field.setRobotPose(odometry.getPoseMeters());
        this.fieldApproximation.setRobotPose(this.poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("accelerometer/x", this.accelerometer.getX());
        SmartDashboard.putNumber("accelerometer/y", this.accelerometer.getY());
        SmartDashboard.putNumber("accelerometer/z", this.accelerometer.getZ());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

        driveSim.setInputs(
                leftController.get() * RobotController.getInputVoltage(),
                rightController.get() * RobotController.getInputVoltage());

        driveSim.update(0.02);
        this.visionSim.processFrame(this.driveSim.getPose());

        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-driveSim.getHeading().getDegrees());
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftController.setVoltage(leftVolts);
        this.rightController.setVoltage(rightVolts);
        this.drive.feed();
    }

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    public void curvatureDrive(double speed, double rotation) {
        this.drive.curvatureDrive(speed, rotation, false);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     * 
     * @param linearSpeed Linear velocity in m/s.
     * @param rotation    Angular velocity in rad/s.
     */
    public void directDrive(double linearSpeed, double rotation) {
        var wheelSpeeds = Constants.DriveTrain.DriveKinematics
                .toWheelSpeeds(new ChassisSpeeds(linearSpeed, 0.0, rotation));
        this.setSpeeds(wheelSpeeds);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
    }

    public void resetEncoders() {
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        this.odometry.resetPosition(Rotation2d.fromRadians(0), 0, 0, pose);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(this.leftEncoder.getRate(), this.rightEncoder.getRate());
    }

    public DifferentialDrive getDrive() {
        return this.drive;
    }

    public Field2d getField() {
        return this.field;
    }

    public Field2d getFieldApproximation() {
        return this.fieldApproximation;
    }

    public ADXRS450_Gyro getGyro() {
        return this.gyro;
    }

    public Accelerometer getAccelerometer() {
        return this.accelerometer;
    }

    public MjpegServer getCameraServer() {
        return this.camServer;
    }

    public UsbCamera getDriveCamera() {
        return this.driveCamera;
    }

    private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = this.feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = this.feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = this.leftPIDController.calculate(this.leftEncoder.getRate(),
                speeds.leftMetersPerSecond);
        final double rightOutput = this.rightPIDController.calculate(this.rightEncoder.getRate(),
                speeds.rightMetersPerSecond);

        this.tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
    }

    private void updateOdometry() {
        this.poseEstimator.update(
                this.gyro.getRotation2d(),
                this.leftEncoder.getDistance(),
                this.rightEncoder.getDistance());

        var visionResult = this.targetCam.getLatestResult();
        if (visionResult.hasTargets()) {
            var imageCaptureTime = visionResult.getTimestampSeconds();
            var target = visionResult.getBestTarget();
            var camToTarget = target.getBestCameraToTarget();
            var targetId = target.getFiducialId();
            var tagPose = this.fieldTags.getTagPose(targetId).get();
            var camPose = tagPose.transformBy(camToTarget.inverse());
            var botPose = camPose.transformBy(Constants.Vision.RobotToTargetCamera.inverse());
            this.poseEstimator.addVisionMeasurement(botPose.toPose2d(), imageCaptureTime);
            this.currentTargetEntry.set(targetId);
        } else {
            this.currentTargetEntry.set(-1);
        }

        this.odometry.update(this.gyro.getRotation2d(), this.leftEncoder.getDistance(),
                this.rightEncoder.getDistance());
    }
}
