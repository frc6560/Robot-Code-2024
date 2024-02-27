// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimbMotor = new CANSparkMax(Constants.CLIMB_MOTOR_LEFT, MotorType.kBrushless);
  private final CANSparkMax rightClimbMotor = new CANSparkMax(Constants.CLIMB_MOTOR_RIGHT, MotorType.kBrushless);
  /** Creates a new Climb. */
  public Climb() {
    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();

    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimbOutputLeft(double output){
    leftClimbMotor.set(-output);
    
  }

  public void setClimbOutputRight(double output){
    rightClimbMotor.set(output);
  }
}
