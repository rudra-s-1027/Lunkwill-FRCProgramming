// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.RunKraken;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Kraken;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final Motor motor1 = new Motor(7);
  // private final Motor motor2 = new Motor(8);

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_Joystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final Kraken kraken = new Kraken(50);
  private final RunKraken krakenCmd = new RunKraken(kraken);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
        m_swerveSubsystem,
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureButtonBindings();

    // Configure the trigger bindings
    configureBindings();
    System.out.println("Robot Container");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // m_Joystick.button(1).onTrue(new RunMotorCommand(motor1, motor2));

    m_Joystick.button(1).onTrue(krakenCmd);
  }

  private void configureButtonBindings() {
    InstantCommand instantCommand = new InstantCommand(() -> {
      m_swerveSubsystem.zeroHeading();
      System.out.println("ZERO HEADING SET");
    });
    new JoystickButton(driverJoystick, 2).whileTrue(instantCommand); // Instant command [executes & immediately
                                                                     // finishes]
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
