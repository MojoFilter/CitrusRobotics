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

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class SoundBoard extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
public static final int Startup_Trigger = 3;
public static final int TankDrive_Trigger = 4;
public static final int Arcade1Stick_Trigger = 5;
public static final int ArcadeSplit_Trigger = 3;
public static final double Trigger_Delay = 0.25;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private DigitalOutput startupOutput = new DigitalOutput(Startup_Trigger);
    private DigitalOutput tankDriveOutput = new DigitalOutput(TankDrive_Trigger);
    private DigitalOutput arcade1StickOutput = new DigitalOutput(Arcade1Stick_Trigger);
    private DigitalOutput arcadeSplitOutput = new DigitalOutput(ArcadeSplit_Trigger);

    /**
    *
    */
    public SoundBoard() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void reset() {
        resetPins(
                startupOutput,
                tankDriveOutput,
                arcade1StickOutput,
                arcadeSplitOutput);
    }

    public void playStartup() {
        this.triggerMomentary(this.startupOutput);
    }

    public void playTankDrive() {
        this.triggerMomentary(this.tankDriveOutput);
    }

    public void playArcade1Stick() {
        this.triggerMomentary(this.arcade1StickOutput);
    }

    public void playArcadeSplit() {
        this.triggerMomentary(this.arcadeSplitOutput);
    }

    private void resetPins(DigitalOutput... pins) {
        for (DigitalOutput pin : pins) {
            pin.set(true);
        }
    }

    private void triggerMomentary(DigitalOutput pin) {
        pin.set(false);
        Commands.waitSeconds(Trigger_Delay)
                .andThen(() -> pin.set(true));
    }
}
