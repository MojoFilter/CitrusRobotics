package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.InputChannels;
import frc.robot.Constants.ArmSettings.ElbowSettings;

public class Elbow extends ArmPart {
    
    public Elbow() {
        super(
            "Elbow",
            CanIds.ElbowMotor, 
            InputChannels.ElbowEncoder,
            ElbowSettings.PositionOffsetDegrees,
            ElbowSettings.StaticGain,
            ElbowSettings.GravityGain,
            ElbowSettings.VelocityGain);
    }

}
