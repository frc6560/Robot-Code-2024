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

public class Stinger extends SubsystemBase {

  final CANSparkMax wristMotor;
  final TalonFX elevatorMotor;
  final CANSparkMax rollerMotor;
  final int MAX_ROTATION = 100;
  final int MIN_ROTATION = 0;

  public Stinger() {
    this.wristMotor = new CANSparkMax(Constants.TRAP_WRIST_MOTOR, MotorType.kBrushless);
    this.elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR);
    this.rollerMotor = new CANSparkMax(Constants.TRAP_FEED_MOTOR, MotorType.kBrushless);

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
    wristMotor.getEncoder().setPosition(angle * Constants.TRAP_WRIST_GEAR_RATIO);
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

  public void setDistanceVelocity(double targetVelocity) {
    targetVelocity /= 2.5;
    final double slowZone = 7;

    if (getExtension() < MIN_ROTATION && targetVelocity < 0) {
      targetVelocity = 0;
    } else if (getExtension() < (MIN_ROTATION + slowZone) && targetVelocity < 0) {
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
    return wristMotor.getEncoder().getPosition() / Constants.TRAP_WRIST_GEAR_RATIO;
  }

  public double getExtension() {
    return elevatorMotor.getPosition().getValue() / Constants.TRAP_ELEVATOR_GEAR_RATIO;
  }

  public double getFeedRate() {
    return rollerMotor.getEncoder().getVelocity();
  }

  public boolean isClearOfShooter() {
    return getAngle() < Constants.TRAP_CLEARANCE_ANGLE;
  }

}
