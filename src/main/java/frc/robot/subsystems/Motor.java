// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
  private final SparkMax motor; // Lunkwill uses SparkMax, WonkoTheSane uses SparkFlex
  private final int ID;
  SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder encoder;

  /** Creates a new Motor. */
  public Motor(int ID) {
    this.ID = ID;
    motor = new SparkMax(ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getSpeed() {
    return motor.get();
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public SparkMax getMotor() {
    return motor;
  }

  public int getID() {
    return this.ID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
