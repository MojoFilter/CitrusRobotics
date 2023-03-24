package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveSettings;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.PlayStartupCommand;
import frc.robot.subsystems.Arm;
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
  private final Arm arm = new Arm();

  // Controllers
  private final XboxController xboxController1 = new XboxController(0);
  private final Joystick joystick = new Joystick(1);

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

    /* 
    this.shoulder.setDefaultCommand(this.commandFactory.getDriveShoulderCommand(() -> -this.joystick.getY()));
    this.elbow.setDefaultCommand(this.commandFactory.getDriveElbowCommand(this.joystick::getTwist));
    this.claw.setDefaultCommand(this.commandFactory.getDriveClawCommand(
        () -> {
          var pov = this.joystick.getPOV();
          if (pov == 0.0) {
            return 1.0;
          } else if (pov == 180.0) {
            return -1.0;
          }
          return 0.0;
        }));
        */


    SmartDashboard.putData("Auto Mode", m_chooser);
    this.dashboardManager.configureDashboard(
        m_driveTrain,
        this.arm,
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
    // Disable twist by default because the arm is sketchy.
    // Create some buttons
    final JoystickButton twistButton = new JoystickButton(xboxController1, 8);
    //twistButton.onTrue(this.commandFactory.getTwistCommand());

    /*
     * final var trackArmButton = new
     * Trigger(()->this.getXboxController1().getLeftTriggerAxis() > 0.5);
     * trackArmButton.onTrue(this.commandFactory.getArmTrackSetpointCommand());
     */
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
    return DriveSettings.MaxSpeed * this.getSpeedGovernor();
  }

  public double getMaxAngularSpeed() {
    return DriveSettings.MaxAngularSpeed;
  }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

  public Arm getArm() {
    return this.arm;
  }

}
