// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
  /*
   * One motor in one swerve module on Lunkwill is for driving, the other is for
   * turning.
   */
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;
  /*
   * Relative Encoder for drive motor b/c we do NOT want the value to persist
   * across power cycles
   */
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  /*
   * Absolute Encoder (CANcoder) for turning motor b/c we DO want the value to
   * persist across power cycles
   */
  private final CANcoder canCoder;
  /*
   * We need to create SparkMaxConfig objects for each motor to set a
   * configuration on the motor
   */
  SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
  SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

  private final PIDController turningPIDController; // PID Controller for turning motor

  private final boolean canCoderReversed;
  private final double canCoderOffsetRad; // Offset = how far the encoder position is from the ACTUAL value

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
      int canCoderID, double canCoderOffset, boolean canCoderReversed) {
    this.canCoderOffsetRad = canCoderOffset;
    this.canCoderReversed = canCoderReversed; // setting the values
    /* instantiating the motor & encoder objects */
    
    canCoder = new CANcoder(canCoderID);
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

    /*
     * Set Configuration to inverted if driveMotorReversed=true, Idle Mode to
     * kBrake. Do same thing for turningMotor.
     */

    driveMotorConfig
        .idleMode(IdleMode.kCoast)
        .inverted(driveMotorReversed);
    turningMotorConfig
        .idleMode(IdleMode.kCoast)
        .inverted(turningMotorReversed);
        
    /*
     * Set the conversion factors for the encoders. conversion factor is how the
     * encoder value gets converted from raw value to a readable number.
     */
    driveMotorConfig.encoder.positionConversionFactor(SwerveModuleConstants.driveEncoderPositionConversionFactor);
    driveMotorConfig.encoder.velocityConversionFactor(SwerveModuleConstants.driveEncoderVelocityConversionFactor);
    turningMotorConfig.encoder.positionConversionFactor(SwerveModuleConstants.rotationEncoderPositionConversionFactor);
    turningMotorConfig.encoder.velocityConversionFactor(SwerveModuleConstants.rotationEncoderVelocityConversionFactor);

    // driveMotorConfig.smartCurrentLimit(10, 20);
    // turningMotorConfig.smartCurrentLimit(10, 20);

    // driveMotorConfig.closedLoopRampRate(0.5);
    // turningMotorConfig.closedLoopRampRate(0.5);

    /* Apply motor config above to the respective motors */
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    /* Get the encoders */
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
    // PID (Propotional, Integral, Derivative) Controller for turning motor. Used to
    // adjust power based on the motor error
    turningPIDController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI); // enableContinuousInput is used to make the PID
                                                                   // controller efficient. -Math.PI to +Math.PI tells
                                                                   // it that our system is circular and calculates the
                                                                   // shortest path, instead of spinning all the way
                                                                   // around.

    

    resetEncoders(); // Reset the encoders to their proper positions
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
    double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRad;

    return angle % (2 * Math.PI);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0); // Set position to 0 for fresh start after robot powered on again
    turningEncoder.setPosition(getAbsoluteEncoderRad()); // Turning encoder's reading aligned w/ the wheel's actual
                                                         // angle
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.15) {
      stop();
      return;
    }

    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    driveMotor.set(desiredState.speedMetersPerSecond / SwerveModuleConstants.kTeleDriveMaxSpeed);
    turningMotor.set(turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
    // SmartDashboard.putString("Swerve[" + turningEncoder.)

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
