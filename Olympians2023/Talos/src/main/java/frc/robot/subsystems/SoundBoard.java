package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Used only for demos, this subsystem allows for managing NT
 * communication to a soundboard client.
 */
public class SoundBoard extends SubsystemBase {

    private final StringPublisher clipPublisher;

    public SoundBoard() {
        var clipTopic = NetworkTableInstance.getDefault().getStringTopic("talos/sfx/clip");
        this.clipPublisher = clipTopic.publish();
        this.setDefaultCommand(
                Commands.waitSeconds(1.0)
                        .andThen(Commands.runOnce(this::clear, this)));
    }

    public void playStartup() {
        this.clipPublisher.set(Constants.SoundBoard.StartupFileName);
    }

    public void reset() {
    }

    public void playTankDrive() {
        this.playStartup();
    }

    public void playArcade1Stick() {
        this.playStartup();
    }

    public void playArcadeSplit() {
        this.playStartup();
    }

    public void clear() {
        this.clipPublisher.set("");
    }
}
