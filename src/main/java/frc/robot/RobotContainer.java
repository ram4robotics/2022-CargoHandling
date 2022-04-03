// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Launcher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeWheels m_intake = new IntakeWheels();
  private final Indexer m_indexer = new Indexer();
  private final Launcher m_launcher = new Launcher();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // private final XboxController m_xbox1 = new XboxController(OIConstants.xbox1_port);
  private final XboxController m_xbox2 = new XboxController(OIConstants.xbox2_port);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the default commands
/*     m_intake.setDefaultCommand(
      new RunCommand(() -> m_intake.stopIntakeWheels(), m_intake)
      );
    m_indexer.setDefaultCommand(
      new RunCommand(() -> m_indexer.stopIndexer(), m_indexer)
    );
    m_launcher.setDefaultCommand(
      new RunCommand(() -> m_launcher.stopLauncher(), m_launcher)
    );
 */  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_xbox2, Button.kLeftBumper.value)
      .whenPressed(() -> m_intake.pullInCargo())
      .whenReleased(() -> m_intake.stopIntakeWheels());
    
    new JoystickButton(m_xbox2, Button.kRightBumper.value)
      .whenPressed(() -> m_intake.pushOutCargo())
      .whenReleased(() -> m_intake.stopIntakeWheels());
    
    // new JoystickButton(m_xbox2, Button.kA.value)
    //   .whenPressed(() -> m_intake.openArm());
    // new JoystickButton(m_xbox2, Button.kB.value)
    //   .whenPressed(() -> m_intake.closeArm());
    
    new JoystickButton(m_xbox2, Button.kX.value)
      .whenPressed(() -> m_indexer.feedToLauncher())
      .whenReleased(() -> m_indexer.stopIndexer());
    new JoystickButton(m_xbox2, Button.kY.value)
      .whenPressed(() -> m_indexer.throwAwayToIntake())
      .whenReleased(() -> m_indexer.stopIndexer());
    
    new JoystickButton(m_xbox2, Button.kA.value)
      .whenPressed(() -> m_launcher.launch())
      .whenReleased(() -> m_launcher.stopLauncher());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return null;
  }
}
