package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.InputChannels;

public class Claw extends ArmPart {
   
    public Claw() {
        super(
            "Claw",
            CanIds.ClawMotor, 
            InputChannels.ClawEncoder);
    }

}
