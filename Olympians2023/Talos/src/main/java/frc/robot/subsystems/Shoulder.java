package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.InputChannels;

public class Shoulder extends ArmPart {
    
    public Shoulder() {
        super(
            "Shoulder",
            CanIds.ShoulderMotor, 
            InputChannels.ShoulderEncoder);
    }   
}
