package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.InputChannels;
import frc.robot.Constants.ArmSettings.ShoulderSettings;

public class Shoulder extends ArmPart {
    
    public Shoulder() {
        super(
            "Shoulder",
            CanIds.ShoulderMotor, 
            InputChannels.ShoulderEncoder,
            ShoulderSettings.PositionOffsetDegrees,
            ShoulderSettings.StaticGain,
            ShoulderSettings.GravityGain,
            ShoulderSettings.VelocityGain);
    }   
}
