
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class AutonomousCommand extends SequentialCommandGroup {


    public AutonomousCommand(DriveTrain driveTrain){
        addCommands(
                // Add Commands here:
                // Also add parallel commands using the
                //
                // addCommands(
                // new command1(argsN, subsystem),
                // parallel(
                // new command2(argsN, subsystem),
                // new command3(argsN, subsystem)
                // )
                // );
                new ExecutePathCommand("Test Path", driveTrain));
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
   
}
