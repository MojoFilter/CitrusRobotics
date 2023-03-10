package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.PlayStartupCommand;
import frc.robot.commands.TwistCommand;
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
  private final SoundBoard m_soundBoard = new SoundBoard();
  private final DriveTrain m_driveTrain = new DriveTrain();

  // Joysticks
  private final XboxController xboxController1 = new XboxController(0);

  private final Rumbler rumbler = new Rumbler();
  private final DashboardManager dashboardManager = new DashboardManager();

  private final CommandFactory commandFactory;

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.commandFactory = new CommandFactory(this);

    // Configure the button bindings
    configureButtonBindings();

    final var arcadeDriveCommand = this.commandFactory.getArcadeDriveCommand();
    // Configure default commands
    m_driveTrain.setDefaultCommand(arcadeDriveCommand);

    // Configure autonomous sendable chooser
    // this should be set up in the dashboard manager
    m_chooser.addOption("Autonomous Command", new AutonomousCommand(m_driveTrain));
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_driveTrain));

    SmartDashboard.putData("Auto Mode", m_chooser);
    this.dashboardManager.configureDashboard(
        m_driveTrain,
        arcadeDriveCommand,
        this.commandFactory.getStickDriveCommand(),
        this.commandFactory.getTankDriveCommand(),
        this.commandFactory.getCurvatureDriveCommand());
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
    twistButton.onTrue(this.commandFactory.getTwistCommand());

    final var chaseButton = new JoystickButton(xboxController1, 4);
    chaseButton.whileTrue(this.commandFactory.getChaseTargetCommand());
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

  public double getMaxSpeed() {
    return Constants.DriveTrain.MaxSpeed * this.getSpeedGovernor();
  }

  public double getMaxAngularSpeed() {
    return Constants.DriveTrain.MaxAngularSpeed;
  }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

}
