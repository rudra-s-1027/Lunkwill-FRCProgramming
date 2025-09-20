// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystickCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  public SwerveJoystickCommand(SwerveSubsystem subsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = subsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;

    this.xLimiter = new SlewRateLimiter(SwerveModuleConstants.kTeleDriveMaxSpeed);
    this.yLimiter = new SlewRateLimiter(SwerveModuleConstants.maxAcceleration);
    this.turnLimiter = new SlewRateLimiter(SwerveModuleConstants.maxAngularAcceleration);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turnSpeed = turningSpdFunction.get();

    //System.out.println("Xspeed: " + xSpeed + "Yspeed: " + ySpeed + "TURNSpeed: " + turnSpeed);

    // 2. Apply a deadband (if the joysticks do not center to exactly 0, ignore any
    // small inputs to protect the motors)
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;

    // 3. Make the driving smoother with a rate limiter to prevent sudden movements
    // of the motors if the joystick is pushed violently
    xSpeed = xLimiter.calculate(xSpeed) * SwerveModuleConstants.kTeleDriveMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveModuleConstants.kTeleDriveMaxSpeed;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveModuleConstants.kTeleDriveMaxAngularSpeed;

    // 4. Construct the desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
          swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] desiredStates = SwerveModuleConstants.kSwerveDriveKinematics
        .toSwerveModuleStates(chassisSpeeds);

    // 6. Send the desired states to the swerve modules
    swerveSubsystem.setModuleStates(desiredStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
