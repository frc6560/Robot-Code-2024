// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimbMotor = new CANSparkMax(Constants.CLIMB_MOTOR_LEFT, MotorType.kBrushless);
  private final CANSparkMax rightClimbMotor = new CANSparkMax(Constants.CLIMB_MOTOR_RIGHT, MotorType.kBrushless);
  
  final SparkLimitSwitch leftLimitSwitch;
  final SparkLimitSwitch rightLimitSwitch;

  final CANSparkMax climbRollerMotor = new CANSparkMax(Constants.CLIMB_DRIVE_MOTOR, MotorType.kBrushless);

  NetworkTableEntry climbOverride = NetworkTableInstance.getDefault().getTable("Climb").getEntry("Climb Override");
  /** Creates a new Climb. */
  public Climb() {
    climbRollerMotor.restoreFactoryDefaults();
    climbRollerMotor.setInverted(false);
    climbRollerMotor.setIdleMode(IdleMode.kBrake);

    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();

    rightClimbMotor.setInverted(true);

    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);

    leftClimbMotor.setSmartCurrentLimit(40);
    rightClimbMotor.setSmartCurrentLimit(40);

    leftClimbMotor.getEncoder().setPosition(0);
    rightClimbMotor.getEncoder().setPosition(0);
    

    leftLimitSwitch = leftClimbMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    leftLimitSwitch.enableLimitSwitch(true);
    

    rightLimitSwitch = rightClimbMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    rightLimitSwitch.enableLimitSwitch(true);
    

    climbOverride.setBoolean(true);

    ntDispTab("Climb")
    .add("Left Position", this::getLeftClimbPos)
    .add("Right Position", this::getRightClimbPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimbOutputLeft(double output){
    if(leftClimbMotor.getEncoder().getPosition() < 0 && !climbOverride.getBoolean(false)){
      output = Math.max(output, 0);
    }
    
    leftClimbMotor.set(output);
    
  }

  public void setClimbOutputRight(double output){
    if(rightClimbMotor.getEncoder().getPosition() < 0 && !climbOverride.getBoolean(false)){
      output = Math.max(output, 0);
    }

    rightClimbMotor.set(output);
  }

  public void setClimbDriveMotor(double output){
    climbRollerMotor.set(output);
  }

  public double getLeftClimbPos(){
    return leftClimbMotor.getEncoder().getPosition();
  }

  public double getRightClimbPos(){
    return rightClimbMotor.getEncoder().getPosition();
  }
}
