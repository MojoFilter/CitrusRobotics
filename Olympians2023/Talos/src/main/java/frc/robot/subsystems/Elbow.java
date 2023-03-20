package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.InputChannels;

public class Elbow extends ArmPart {
    
    public Elbow() {
        super(
            "Elbow",
            CanIds.ElbowMotor, 
            InputChannels.ElbowEncoder);
    }

}
