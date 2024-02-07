// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  final Limelight limelight;
  final Trap trap;

  final TalonFX arcMotor;
  final TalonFX shooterMotorRight;
  final TalonFX shooterMotorLeft;

  final CANSparkMax feedMotor;

  final ColorSensorV3 colorSensor;

  double targetRPM = 0;
  double targetAngle = 0;

  public Shooter(Limelight limelight, Trap trap) {
    this.limelight = limelight;
    this.trap = trap;

    arcMotor = new TalonFX(Constants.ARC_MOTOR);
    shooterMotorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
    shooterMotorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);

    feedMotor = new CANSparkMax(Constants.SHOOTER_FEED_MOTOR, MotorType.kBrushless);

    colorSensor = new ColorSensorV3(Constants.SHOOTER_COLOR_SENSOR_ID);
  }

  @Override
  public void periodic() {
  }

  public void setRPM(double speed){
    targetRPM = speed;
    shooterMotorRight.set(0); // TODO: Fix this to pid controlled
    shooterMotorLeft.set(0);
  }

  public void setVerticalAngle(double angle){
    targetAngle = angle;
    arcMotor.setPosition(angle * Constants.SHOOTER_ARC_GEAR_RATIO);
  }

  public boolean readyToShoot(){
    return (
      Math.abs(getShooterRPM() - targetRPM) < Constants.SHOOTER_ACCEPTABLE_RPM_DIFF &&
      Math.abs(getShooterVerticalAngle() - targetAngle) < Constants.SHOOTER_ACCEPTABLE_VERTICAL_DIFF &&
      limelight.hasTarget() && Math.abs(limelight.getHorizontalAngle()) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF && 
      trap.isClearOfShooter()
    );
  }


  public boolean getShooterSensorTriggered(){
    return colorSensor.getProximity() > Constants.SENSOR_TRIGGER_PROXIMITY_VALUE;
  }

  public double getShooterRPM(){
    return shooterMotorLeft.getVelocity().getValueAsDouble();
  }

  public double getShooterVerticalAngle(){
    return arcMotor.getPosition().getValueAsDouble();
  }
}
