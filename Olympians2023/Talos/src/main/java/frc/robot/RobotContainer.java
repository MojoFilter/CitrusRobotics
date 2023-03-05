package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.PlayStartupCommand;
import frc.robot.commands.TankDrive;
import frc.robot.commands.Twist;
import frc.robot.subsystems.DashboardManager;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SoundBoard;
import frc.robot.util.Rumbler;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final SoundBoard m_soundBoard = new SoundBoard();
  public final DriveTrain m_driveTrain = new DriveTrain();

  // Joysticks
  private final XboxController xboxController1 = new XboxController(0);

  private final Rumbler rumbler = new Rumbler();
  private final DashboardManager dashboardManager = new DashboardManager();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driveTrain.setDefaultCommand(new ArcadeDrive(() -> -getXboxController1().getRawAxis(1),
        () -> -getXboxController1().getRawAxis(4), "Split", "B260:100 P260 B260:100", m_driveTrain));

    // Configure autonomous sendable chooser
    // this should be set up in the dashboard manager
    m_chooser.addOption("Autonomous Command", new AutonomousCommand(m_driveTrain));
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_driveTrain));

    SmartDashboard.putData("Auto Mode", m_chooser);
    this.dashboardManager.configureDashboard(
        m_driveTrain,
        getArcadeSplitCommand(),
        getArcade1StickCommand(),
        getTankDriveCommand());
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    final JoystickButton twistButton = new JoystickButton(xboxController1, 8);
    twistButton.onTrue(new Twist(m_driveTrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  public void rumble(String rumblePattern) {
    this.rumbler.rumble(this.getXboxController1(), rumblePattern);
  }

  public XboxController getXboxController1() {
    return xboxController1;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void resetIO(boolean announce) {
    m_soundBoard.reset();
    if (announce) {
      new PlayStartupCommand(m_soundBoard).schedule();
    } else {
      m_soundBoard.clear();
    }
  }

  public void playTankDrive() {
    m_soundBoard.playTankDrive();
  }

  public void playArcade1Stick() {
    m_soundBoard.playArcade1Stick();
  }

  public void playArcadeSplit() {
    m_soundBoard.playArcadeSplit();
  }

  public double getSpeedGovernor() {
    return this.getXboxController1().getRightTriggerAxis() > 0.9 ? 1.0
        : this.dashboardManager.getSpeedGovernor();
  }

  private Command getArcadeSplitCommand() {
    return new ArcadeDrive(
        () -> -this.getXboxController1().getRawAxis(1),
        () -> -this.getXboxController1().getRawAxis(4),
        "Split",
        Constants.RumblePatterns.ArcadeSplit,
        m_driveTrain)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  private Command getArcade1StickCommand() {
    return new ArcadeDrive(
        () -> -getXboxController1().getRawAxis(1),
        () -> -getXboxController1().getRawAxis(0),
        "1-Stick",
        Constants.RumblePatterns.Arcade1Stick,
        m_driveTrain)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  private Command getTankDriveCommand() {
    return new TankDrive(
        () -> -getXboxController1().getRawAxis(1),
        () -> -getXboxController1().getRawAxis(5),
        m_driveTrain)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
}
