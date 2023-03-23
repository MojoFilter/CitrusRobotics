package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
import frc.robot.Constants.CanIds;
import frc.robot.Constants.DriveSettings;
import frc.robot.Constants.InputChannels;

/**
 *
 */
public class DriveTrain extends SubsystemBase {

    private CANSparkMax left1;
    private CANSparkMax left2;
    private MotorControllerGroup leftController;
    private CANSparkMax right1;
    private CANSparkMax right2;
    private MotorControllerGroup rightController;
    private DifferentialDrive drive;
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private final AHRS navx;
    private final ADXRS450_Gyro gyro;
    private final BuiltInAccelerometer accelerometer;
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
        this.left1 = new CANSparkMax(CanIds.LeftFrontMotor, MotorType.kBrushed);
        this.left2 = new CANSparkMax(CanIds.LeftRearMotor, MotorType.kBrushed);
        this.leftController = new MotorControllerGroup(left1, left2);
        this.leftController.setInverted(DriveSettings.IsLeftDriveInverted);
        this.addChild("Left Controller", leftController);

        this.right1 = new CANSparkMax(CanIds.RightFrontMotor, MotorType.kBrushed);
        this.right2 = new CANSparkMax(CanIds.RightRearMotor, MotorType.kBrushed);
        this.rightController = new MotorControllerGroup(right1, right2);
        this.rightController.setInverted(DriveSettings.IsRightDriveInverted);
        this.addChild("Right Controller", rightController);

        this.drive = new DifferentialDrive(leftController, rightController);
        this.addChild("Drive", drive);
        this.drive.setSafetyEnabled(true);
        this.drive.setExpiration(0.1);
        this.drive.setMaxOutput(1.0);

        this.leftEncoder = new Encoder(
                InputChannels.DriveLeftEncoderChannelA,
                InputChannels.DriveLeftEncoderChannelB,
                DriveSettings.IsLeftDriveInverted,
                EncodingType.k4X);
        this.addChild("Left Encoder", leftEncoder);
        this.leftEncoder.setDistancePerPulse(DriveSettings.EncoderDistancePerPulse);

        this.rightEncoder = new Encoder(
                InputChannels.DriveRightEncoderChannelA,
                InputChannels.DriveRightEncoderChannelB,
                DriveSettings.IsRightEncoderInverted,
                EncodingType.k4X);
        this.addChild("Right Encoder", rightEncoder);
        this.rightEncoder.setDistancePerPulse(DriveSettings.EncoderDistancePerPulse);

        this.gyro = new ADXRS450_Gyro();
        this.addChild("Gyro", this.gyro);

        this.navx = new AHRS(SPI.Port.kMXP);
        this.addChild("Navx", this.navx);

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

        this.resetEncoders();

        this.field = new Field2d();
        this.fieldApproximation = new Field2d();

        this.odometry = new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(getHeading()),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());

        this.poseEstimator = new DifferentialDrivePoseEstimator(
                DriveSettings.DriveKinematics,
                this.gyro.getRotation2d(),
                this.leftEncoder.getDistance(),
                this.rightEncoder.getDistance(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        this.feedforward = new SimpleMotorFeedforward(DriveSettings.StaticGainVolts, DriveSettings.VelocityGainVolts);
        this.leftPIDController = new PIDController(DriveSettings.LinearP, DriveSettings.LinearI, DriveSettings.LinearD);
        this.rightPIDController = new PIDController(DriveSettings.LinearP, DriveSettings.LinearI,
                DriveSettings.LinearD);

        try {
            this.fieldTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            /*
             * List<AprilTag> testTags = new ArrayList<AprilTag>();
             * testTags.add(new AprilTag(6, new Pose3d(8, 3, 1, new Rotation3d())));
             * var length = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
             * var width = Units.feetToMeters(26) + Units.inchesToMeters(3.5);
             * this.fieldTags = new AprilTagFieldLayout(testTags, length, width);
             */
            this.visionSim.addVisionTargets(this.fieldTags);
        } catch (Exception ioe) {// IOException ioe) {
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
        var wheelSpeeds = DriveSettings.DriveKinematics
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

    public Gyro getGyro() {
        return this.navx;
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

    public AHRS getNav() {
        return this.navx;
    }

    public Encoder getLeftEncoder() {
        return this.leftEncoder;
    }

    public Encoder getRightEncoder() {
        return this.rightEncoder;
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
