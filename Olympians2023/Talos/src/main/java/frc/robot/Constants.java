package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrain {
        public static final double WheelDiameterMeters = 0.15;
        public static final int EncoderCPR = 1024;
        public static final double EncoderDistancePerPulse = (WheelDiameterMeters * Math.PI) / (double) EncoderCPR;

        public static final double DefaultSpeedGovernor = 0.65;
        public static final double MaxSpeed = 3.0; // m/s
        public static final double MaxAngularSpeed = 2 * Math.PI; // rad/s

        public static final double TrackWitdhMeters = 0.69;
        public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(
                TrackWitdhMeters);

        // THESE VALUES ARE BOGUS AND ONLY HERE FOR SIMULATION. SHOULD NOT BE USED ON
        // LIVE ROBOT
        // Real values should be determined using the Robot Characterization Toolsuite
        public static final double StaticGainVolts = 0.22;
        public static final double VelocityGainVolts = 1.98;
        public static final double AccelerationGainVolts = 0.2;
        public static final double PDriveVelocity = 8.5;

        public static final double GyroSensitivity = 0.007;

        // units per second
        public static final double SpeedRateLimit = 3.0;
        public static final double RotationRateLimit = 3.0;

        // PID gain values
        public static final double LinearP = 0.1;
        public static final double LinearD = 0.0;
        public static final double AngularP = 0.1;
        public static final double AngularD = 0.0;
    }

    public static final class Vision {
        public static final String TargetCameraName = "TargetCam";
        public static final double TargetCameraOffsetMetersX = 1;
        public static final double TargetCameraOffsetMetersY = 1;
        public static final double TargetCameraOffsetMetersZ = 1;
        public static final double TargetCameraRoll = 0;
        public static final double TargetCameraPitch = 0;
        public static final double TargetCameraYaw = 0;
        public static final double TargetCameraFOVDegrees = 75.0;
        public static final double TargetCameraMaxLedRangeMeters = 20; // Not really relevant

        // these are all in pixels
        public static final int TargetCameraResolutionWidth = 640;
        public static final int TargetCameraResolutionHeight = 480;
        public static final double TargetCameraMinTargetArea = 10; // square pixels

        public static final Translation3d TargetCameraPosition = new Translation3d(TargetCameraOffsetMetersX,
                TargetCameraOffsetMetersY, TargetCameraOffsetMetersZ);

        public static final Rotation3d TargetCameraRotation = new Rotation3d(TargetCameraPitch, TargetCameraRoll,
                TargetCameraYaw);

        public static final Transform3d RobotToTargetCamera = new Transform3d(TargetCameraPosition,
                TargetCameraRotation);

        public static final double ChaseTargetHeightMeters = Units.feetToMeters(4);
        public static final double ChaseTargetRangeMeters = 1.5;

    }

    public static final class ArmSettings {
        public static final double UpperArmLengthMeters = Units.inchesToMeters(30);
        public static final double UpperArmMassKgs = 8.0;

        public static final class Shoulder {
            public static final int MotorPort = 5;
            public static final int EncoderAChannel = 6;
            public static final int EncoderBChannel = 7;

            public static final double EncoderDistancePerPulse = 2.0 * Math.PI / 4096;
            public static final double GearReduction = 200;
            public static final double MinAngleRads = Units.degreesToRadians(-75);
            public static final double MaxAngleRads = Units.degreesToRadians(255);

            public static final String PositionKey = "ShoulderPosition";
            public static final double DefaultPositionDegrees = 75;

            public static final String PKey = "ShoulderP";
            public static final double DefaultP = 50.0;

            public static final double DefaultI = 0.0;
            public static final double DefaultD = 0.0;

            public static final double MaxVelocityRadsPerSecond = 3.0;
            public static final double MaxAccelerationMetersPerSecondSquared = 10.0;

            // THESE NEED REAL VALUES FROM SYSTEM IDENTIFICATION
            public static final double SVolts = 1;
            public static final double GVolts = 1;
            public static final double VVoltSecondPerRad = 0.5;
            public static final double AVoltSecondSquaredPerRad = 0.1;
        }
    }

    public static final class SoundBoard {
        public static String StartupFileName = "ready_to_roll.ogg";
    }

    public static final class Auto {
        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 1;

        public static final double RamseteB = 2;
        public static final double RamseteZ = 0.7;
    }

    public static final class RumblePatterns {
        public static final String ArcadeSplit = "B260:100 P260 B260:100";
        public static final String Arcade1Stick = "B500:100";
        public static final String TankDrive = "L500:75 R500:75";
    }

    public static final class Dashboard {
        public static String DriveTabName = "Drive";
        public static String TestTabName = "Testing";
        public static final String DriveModeTitle = "Drive Mode";        

        public static final class DriveModeOptions {
            public static final String ArcadeSplit = "Arcade Split";
            public static final String Arcade1Stick = "Arcade 1-Stick";
            public static final String Tank = "Tank Drive";
            public static final String Curvature = "Curvature Drive";
        }

        public static final String SpeedGovernorTitle = "Speed Governor";
        public static final String TargetCameraTitle = "Target Camera";
        public static final String TargetCameraName = "TargetCam";
        public static final String DriveCameraTitle = "Drive Camera";
        public static final String DriveCameraName = "DriveCam";
        public static final String[] DriveCameraUrls= {"http://10.93.0.2:1181/stream.mjpg"};
        public static final String[] TargetCameraUrls = {"http://photonvision:1184/stream.mjpg"};
    }

    public static final class Balance {
        public static final double RAD2DEG = 180 / Math.PI;
        public static final double TicksPerSecond = 50;

        // Angle in degrees at which Talos recognizes that it's moving onto the charge station
        public static double OnChargeStationDegrees = 13.0;

        // Angle in degrees at which Talos can assum it's level on the charging station.
        public static double LevelDegrees = 5.0;

        // The buffer time on sensor readings to filter out noise
        public static double DebounceTime = 0.2;

        public static double TalosSpeedFast = 0.4;
        public static double TalosSpeedSlow = 0.2;

        // Time in seconds to drive back toward the scoring zone from the start position
        public static double FirstTapTime = 0.4;

        // Time in seconds to back up after knocking over the cone before securing it with
        // a second tap
        public static double ScoringBackUpTime = 0.2;

        // Amount of time to drive forward, securing the knocked-off cone in the score zone
        public static double SecondTapTime = 0.3;
    }
}
