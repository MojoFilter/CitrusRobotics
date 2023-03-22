package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.InputChannels;
import frc.robot.Constants.ArmSettings.ClawSettings;

public class Claw extends ArmPart {
   
    public Claw() {
        super(
            "Claw",
            CanIds.ClawMotor, 
            InputChannels.ClawEncoder,
            ClawSettings.PositionOffsetDegrees,
            ClawSettings.StaticGain,
            ClawSettings.GravityGain,
            ClawSettings.VelocityGain);
    }

}
