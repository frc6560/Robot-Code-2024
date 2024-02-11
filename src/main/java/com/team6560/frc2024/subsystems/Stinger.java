// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.Constants.StingerConstants;;

public class Stinger extends SubsystemBase {

  final CANSparkMax wristMotor;
  final TalonFX elevatorMotor;
  final CANSparkMax rollerMotor;
  final int MAX_ROTATION = 100;
  final int MIN_ROTATION = 0;

  public Stinger() {
    this.wristMotor = new CANSparkMax(StingerConstants.STINGER_WRIST_ID, MotorType.kBrushless);
    this.elevatorMotor = new TalonFX(StingerConstants.STINGER_ELEVATOR_ID);
    this.rollerMotor = new CANSparkMax(StingerConstants.STINGER_ROLLERS_ID, MotorType.kBrushless);

    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());

    var elevatorPIDConfig = new Slot0Configs();
    elevatorPIDConfig.kS = 0.04;
    elevatorPIDConfig.kV = 0.1;
    elevatorPIDConfig.kP = 0.1;
    elevatorPIDConfig.kI = 0;
    elevatorPIDConfig.kD = 0;

    elevatorMotor.getConfigurator().apply(elevatorPIDConfig);
  }

  @Override
  public void periodic() {
  }

  public void setAngle(double angle) {
    wristMotor.getEncoder().setPosition(angle * StingerConstants.WRIST_GEAR_RATIO);
  }

  public void setElevatorPos(double targetPosRotation) {

    if (targetPosRotation > MAX_ROTATION) {
      targetPosRotation = MAX_ROTATION;
    } else if (targetPosRotation < MIN_ROTATION) {
      targetPosRotation = MIN_ROTATION;
    }

    elevatorMotor.setPosition(targetPosRotation * Constants.TALONFX_POS_TO_ROTATION);
    // VerticalMotorRight.set(TalonFXControlMode.Position, targetPosRotation);
  }

  public void setElevatorVelocity(double targetVelocity) {
    targetVelocity /= 2.5;
    final double slowZone = 7;

    if (getExtension() < MIN_ROTATION && targetVelocity < 0) {
      targetVelocity = 0;
    } else if (getExtension() < slowZone && targetVelocity < 0) {
      targetVelocity = Math.max(targetVelocity, -0.25);
    }

    if (getExtension() > MAX_ROTATION && targetVelocity > 0) {
      targetVelocity = 0;
    } else if (getExtension() > (MAX_ROTATION - slowZone) && targetVelocity > 0) {
      targetVelocity = Math.min(targetVelocity, 0.25);
    }

    elevatorMotor.set(targetVelocity);
  }

  public void setRoller(double output) {
    rollerMotor.set(output);
  }

  public double getAngle() {
    return wristMotor.getEncoder().getPosition() / StingerConstants.WRIST_GEAR_RATIO;
  }

  public double getExtension() {
    return elevatorMotor.getPosition().getValue();
  }

  public double getFeedRate() {
    return rollerMotor.getEncoder().getVelocity();
  }

  public boolean isClearOfShooter() {
    return getAngle() > StingerConstants.TRAP_CLEARANCE_ANGLE;
  }

}
