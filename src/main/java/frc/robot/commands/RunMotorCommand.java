// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunMotorCommand extends Command {
  private final Motor motor1;
  private final Motor motor2;
  private final Timer timer;

  /** Creates a new RunMotor Command. */
  public RunMotorCommand(Motor sub1, Motor sub2) {
    motor1 = sub1;
    motor2 = sub2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor1);
    addRequirements(motor2);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    motor1.stop();
    motor2.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor1.move(1);
    motor2.move(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor1.stop();
    motor2.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 2.0;
  }
}
