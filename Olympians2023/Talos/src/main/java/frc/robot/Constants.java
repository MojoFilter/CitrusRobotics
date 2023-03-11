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
        public static final String[] DriveCameraUrls = { "http://10.93.0.2:1181/stream.mjpg" };
        public static final String[] TargetCameraUrls = { "http://photonvision.local/" };
    }
}
