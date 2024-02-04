// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.team6560.frc2024.Constants;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {

  private CANSparkMax transferMotor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /** Creates a new Transfer. */
  public Transfer() {
    transferMotor = new CANSparkMax(Constants.TRANSFER_MOTOR, MotorType.kBrushless);
    transferMotor.restoreFactoryDefaults();
    transferMotor.setIdleMode(IdleMode.kBrake);
    transferMotor.setSmartCurrentLimit(25);
  }

   public void setSpeed(double speed) {
    transferMotor.set(speed);
  }

  public boolean isInProximity() {
    return m_colorSensor.getProximity() < 20;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
