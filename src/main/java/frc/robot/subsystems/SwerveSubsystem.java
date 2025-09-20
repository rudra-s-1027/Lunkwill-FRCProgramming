// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
      SwerveDriveConstants.kFrontLeftDriveMotorPort,
      SwerveDriveConstants.kFrontLeftTurningMotorPort,
      SwerveDriveConstants.kFrontLeftDriveEncoderReversed,
      SwerveDriveConstants.kFrontLeftTurningEncoderReversed,
      SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(
      SwerveDriveConstants.kFrontRightDriveMotorPort,
      SwerveDriveConstants.kFrontRightTurningMotorPort,
      SwerveDriveConstants.kFrontRightDriveEncoderReversed,
      SwerveDriveConstants.kFrontRightTurningEncoderReversed,
      SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
      SwerveDriveConstants.kBackLeftDriveMotorPort,
      SwerveDriveConstants.kBackLeftTurningMotorPort,
      SwerveDriveConstants.kBackLeftDriveEncoderReversed,
      SwerveDriveConstants.kBackLeftTurningEncoderReversed,
      SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
      SwerveDriveConstants.kBackRightDriveMotorPort,
      SwerveDriveConstants.kBackRightTurningMotorPort,
      SwerveDriveConstants.kBackRightDriveEncoderReversed,
      SwerveDriveConstants.kBackRightTurningEncoderReversed,
      SwerveDriveConstants.kBackRightDriveAbsoluteEncoderPort,
      SwerveDriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      SwerveDriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public static final String ANSI_RESET = "\u001B[0m";
  public static final String ANSI_RED = "\u001B[31m";
  // private Timer gyroResetTimer = new Timer();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
        String errorMsg = ANSI_RED + "ERROR RESETING GYRO: " + e.getMessage() + "\n" + "Stack Trace: \n" + ANSI_RESET;
        System.out.println(errorMsg);
        e.printStackTrace();
      }
    }).start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public AHRS getGyro() {
    return gyro;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());

    double[] swerveDrive = new double[] {
        frontLeft.getDrivePosition(), frontRight.getDrivePosition(),
        backLeft.getDrivePosition(), backRight.getDrivePosition(),
        frontLeft.getTurningPosition(), frontRight.getTurningPosition(),
        backLeft.getTurningPosition(), backRight.getTurningPosition()
    };

    SmartDashboard.putNumberArray("Swerve Drive", swerveDrive);
  }

  /* Stop all motor modules */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.kTeleDriveMaxSpeed); // normalizes
                                                                                                            // wheel
                                                                                                            // speed to
                                                                                                            // avoid
                                                                                                            // losing
                                                                                                            // control
                                                                                                            // of robot
                                                                                                            // steering

    // System.out.println("State 1: " + desiredStates[0]);
    // System.out.println("State 2: " + desiredStates[1]);
    // System.out.println("State 3: " + desiredStates[2]);
    // System.out.println("State 4: " + desiredStates[3]);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
