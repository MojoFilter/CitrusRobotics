package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;

public class Rumbler {
    
    public Rumbler()
    {
    }

    public void rumble(GenericHID controller, String pattern)
    {
        // split the pattern into individual instruction parts which can be 
        // used to build a command that can can be scheduled.
        String[] patternParts = pattern.split(" ");

        Command cmd = Commands.runOnce(()-> controller.setRumble(RumbleType.kBothRumble, 0.0));

        for (String part : patternParts) {
            // decode part
            Command nextCommand;
            
            nextCommand = 
                Commands.runOnce(()->System.out.println("Rumble command: " + part))
                        .andThen(Commands.runOnce(()->controller.setRumble(RumbleType.kBothRumble, 1.0)))
                        .andThen(Commands.waitSeconds(1.0));
            
            cmd = cmd.andThen(nextCommand);
        }

        // make sure vibration is off when the pattern completes
        cmd = cmd.finallyDo(interrupted -> controller.setRumble(RumbleType.kBothRumble, 0));
        cmd.schedule();
    }
}
