// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveModuleConstants {
    // Diameter of the wheel in meters, converted using `Units.inchesToMeters`
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);

    // Given Motor Rotations, convert to Meters traveled
    // (1 rev / Gear Ratio) * ((2 * PI * r) / (1 Rev)) =
    // (2 * PI * r) / (Gear Ratio) =
    public static final double driveEncoderPositionConversionFactor = 0.0540992905;

    // dx/dt
    // Given RPM, convert to m/s
    public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;

    // Given Motor Rotations, convert to Radians travelled
    // (1 rev / Gear Ratio) * ((2 * PI) RAD / (1 Rev))
    // (2 * PI) RAD / (Gear Ratio)
    public static final double rotationEncoderPositionConversionFactor = 0.335103217;

    // Given RPM, convert to radians/seconds
    public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

    public static final double kPTurning = 0.5;
    // Global
    public static final double maxSpeed = 5; // meters/sec
    public static final double maxAcceleration = 10; // meters/sec^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
    // Teleop max speeds
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0;
    public static final double kTeleDriveMaxAngularSpeed = 3;

    public static final double kTrackWidth = Units.inchesToMeters(21.25);
    public static final double kWheelBase = Units.inchesToMeters(21.125);

    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // front left (-,+)
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front right (+,+)
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0), // back left (-,-)
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0) // back right (+,-)
    );
  }

  public static final class SwerveDriveConstants {
    /* MOTOR VARIABLE DECLARATIONS */

    /* FRONT LEFT MOTORS */
    public static final int kFrontLeftDriveMotorPort = 1; // drive motor port
    public static final int kFrontLeftTurningMotorPort = 2; // turning motor port
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 11; // absolute encoder (CANcoder) port
    public static final boolean kFrontLeftTurningEncoderReversed = true; // drive encoder
    public static final boolean kFrontLeftDriveEncoderReversed = false; // turning encoder
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

    /* BACK LEFT MOTORS */
    public static final int kBackLeftDriveMotorPort = 5; // drive motor port
    public static final int kBackLeftTurningMotorPort = 6; // turning motor port
    public static final int kBackLeftDriveAbsoluteEncoderPort = 14; // absolute encoder (CANcoder) port
    public static final boolean kBackLeftTurningEncoderReversed = true; // turning encoder
    public static final boolean kBackLeftDriveEncoderReversed = false; // drive encoder
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

    /* FRONT RIGHT MOTORS */
    public static final int kFrontRightDriveMotorPort = 8; // drive motor port
    public static final int kFrontRightTurningMotorPort = 7; // turning motor port
    public static final int kFrontRightDriveAbsoluteEncoderPort = 12; // absolute encoder (CANcoder) port
    public static final boolean kFrontRightTurningEncoderReversed = true; // turning encoder
    public static final boolean kFrontRightDriveEncoderReversed = true; // drive encoder
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

    /* BACK RIGHT MOTORS */
    public static final int kBackRightDriveMotorPort = 4; // drive motor port
    public static final int kBackRightTurningMotorPort = 3; // turning motor port
    public static final int kBackRightDriveAbsoluteEncoderPort = 13; // absolute encoder (CANcoder) port
    public static final boolean kBackRightTurningEncoderReversed = true; // turning encoder
    public static final boolean kBackRightDriveEncoderReversed = true; // drive encoder
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    /* END MOTORS */

    /* ABSOLUTE ENCODER (CANCoders) OFFSETS */
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.867188 * 2 * Math.PI;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.245361 * 2 * Math.PI;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.038330 * 2 * Math.PI;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.473633 * 2 * Math.PI;
  }

  public static final class OIConstants {
    public static final double kDeadband = 0.15;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 2;
    public static final int kDriverFieldOrientedButtonIdx = 1;
  }
}
