// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.RunIntakeandIndex;
import frc.robot.commands.runIndexer;
import frc.robot.commands.runIntake;
import frc.robot.commands.shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final LEDS m_leds = new LEDS();
  public final Intake m_spinIntake = new Intake();
  public final Indexer m_spinIndex = new Indexer();
  public final Shooter m_spinShooter = new Shooter();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  // XboxController m_codriverController = new XboxController(OIConstants.kCodriverControllerPort);
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

  // A chooser for autonomous commands
  private final SendableChooser<Command> m_autoChooser;
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leds.ledOn(0, 0, 255);

    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
  m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () ->
              m_robotDrive.drive(
                  // m_xspeedLimiter.calculate( -0.1 )*SwerveConstants.kMaxSpeedTeleop,
                  // m_yspeedLimiter.calculate( -0.0 )*SwerveConstants.kMaxSpeedTeleop,
                  // m_rotLimiter.calculate( -0.0 )*ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                  m_xspeedLimiter.calculate( -m_driverController.getLeftY() )*SwerveConstants.kMaxSpeedTeleop,
                  m_yspeedLimiter.calculate( -m_driverController.getLeftX() )*SwerveConstants.kMaxSpeedTeleop,
                  m_rotLimiter.calculate( -m_driverController.getRightX() )*ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                  false), //field orintation stuffs
          m_robotDrive));

    m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
      // m_driver.leftTrigger().whileTrue(new AmpShoot(m_climber, m_shooter, m_index, m_leds));
      // new JoystickButton(m_driverController, Button.kA.value)
      // .whileTrue(new ClimbDown(m_climber));
      // new POVButton(m_driverController, 180)
      // .whileTrue(new InstantCommand(() -> m_shooter.stopShooter()));
      new JoystickButton(m_driverController, Button.kBack.value)
      .whileTrue(new RunCommand(() -> m_robotDrive.zeroIMU()));
      new JoystickButton(m_driverController, Button.kLeftStick.value)
      .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.toggleMaxOutput()));
      new JoystickButton(m_driverController, Button.kB.value)
      .whileTrue(new runIntake(m_spinIntake));
      new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(new runIndexer(m_spinIndex));
       new JoystickButton(m_driverController, Button.kX.value)
      .whileTrue(new shoot(m_spinShooter));
      new JoystickButton(m_driverController, Button.kY.value)
      .whileTrue(new RunIntakeandIndex(m_spinIntake,m_spinIndex));

  }
 public Command getAutonomousCommand() {return m_autoChooser.getSelected();}
}