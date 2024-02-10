// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2024.Constants;

public class Climb extends SubsystemBase {

  private CANSparkMax rightClimbMotor; 
  private CANSparkMax leftClimbMotor; 

  /** Creates a new Climb. */
  public Climb() {
    rightClimbMotor = new CANSparkMax(Constants.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
    rightClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setSmartCurrentLimit(25);

    leftClimbMotor = new CANSparkMax(Constants.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
    leftClimbMotor.restoreFactoryDefaults();
    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    leftClimbMotor.setSmartCurrentLimit(25);
    
    rightClimbMotor.follow(leftClimbMotor); 

 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
