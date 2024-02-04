// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(15, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(25);
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
