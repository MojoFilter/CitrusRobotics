package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SoundBoard;

/**
 *
 */
public class PlayStartupCommand extends InstantCommand {

    private final SoundBoard m_soundBoard;

    public PlayStartupCommand(SoundBoard subsystem) {
        m_soundBoard = subsystem;
        addRequirements(m_soundBoard);
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_soundBoard.playStartup();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
