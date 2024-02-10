// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

// import static com.team6560.frc2024.Constants.CLIMB_TOP_SOFT_LIMITS;
// import static com.team6560.frc2024.Constants.LEFT_CLIMB_MOTOR;

// import static com.team6560.frc2024.Constants;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.utility.NetworkTable.NtValueDisplay;

public class Climb extends SubsystemBase {

  private CANSparkMax rightClimbMotor;
  private CANSparkMax leftClimbMotor;

  SparkPIDController rightClimbMotorPID;
  SparkPIDController leftClimbMotorPID;

  /** Creates a new Climb. */
  public Climb() {
    rightClimbMotor = new CANSparkMax(Constants.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
    rightClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setSmartCurrentLimit(25);
    rightClimbMotorPID = rightClimbMotor.getPIDController();
    rightClimbMotorPID.setP(0.0);
    rightClimbMotorPID.setI(0.0);
    rightClimbMotorPID.setD(0.0);
    rightClimbMotorPID.setIZone(0.0);
    rightClimbMotorPID.setFF(0.0);
    rightClimbMotorPID.setOutputRange(Constants.CLIMB_MIN_VERTICAL_ROTATION, Constants.CLIMB_MAX_VERTICAL_ROTATION);

    leftClimbMotor = new CANSparkMax(Constants.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
    leftClimbMotor.restoreFactoryDefaults();
    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    leftClimbMotor.setSmartCurrentLimit(25);
    leftClimbMotorPID = rightClimbMotor.getPIDController();
    leftClimbMotorPID.setI(0.0);
    leftClimbMotorPID.setD(0.0);
    leftClimbMotorPID.setIZone(0.0);
    leftClimbMotorPID.setFF(0.0);
    leftClimbMotor.follow(rightClimbMotor);

    NtValueDisplay.ntDispTab("Climb")
        .add("Left Climb Vel", this::getleftVelocity)
        .add("Right Climb Vel", this::getRightVelocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    rightClimbMotor.set(speed);
  }

  public void setHeight(double targetPosRotation) {
    rightClimbMotorPID.setReference(targetPosRotation, ControlType.kPosition);
  }

  public double getleftVelocity() {
    return leftClimbMotor.getEncoder().getVelocity();
  }

  public double getRightVelocity() {
    return rightClimbMotor.getEncoder().getVelocity();
  }

}
