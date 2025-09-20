// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kraken extends SubsystemBase {
  private TalonFX kraken;
  private final DutyCycleOut controlRequest;

  /** Creates a new Kraken. */
  public Kraken(int motorID) {
    kraken = new TalonFX(motorID);
    controlRequest = new DutyCycleOut(0);
    kraken.setControl(controlRequest);
  }

  public void set(double speed) {
    kraken.set(speed);
  }

  public void stopMotor() {
    kraken.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
