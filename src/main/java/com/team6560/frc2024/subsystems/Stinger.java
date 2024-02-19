// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
// import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.Constants.StingerConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Stinger extends SubsystemBase {

  final CANSparkMax wristMotor; 
  final TalonFX elevatorMotor;
  final CANSparkMax rollerMotor;

  final SparkAnalogSensor rollerLimitSwitch;

  final double MAX_ROTATION = 8.0390625; //elevator
  final double MIN_ROTATION = 0.4;
  private MotionMagicVoltage m_elevatorRequest;

  public Stinger() {
    this.wristMotor = new CANSparkMax(StingerConstants.STINGER_WRIST_ID, MotorType.kBrushless);
    this.elevatorMotor = new TalonFX(StingerConstants.STINGER_ELEVATOR_ID);
    this.rollerMotor = new CANSparkMax(StingerConstants.STINGER_ROLLERS_ID, MotorType.kBrushless);

    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());

    var elevatorPIDConfig = new TalonFXConfiguration();

    var elevatorSlot0Configs = elevatorPIDConfig.Slot0;
    elevatorSlot0Configs.kS = 0.03;
    elevatorSlot0Configs.kV = 0.2;
    elevatorSlot0Configs.kP = 2;
    elevatorSlot0Configs.kI = 0;
    elevatorSlot0Configs.kD = 0;
    
    var elevatorMotionMagicConfig = elevatorPIDConfig.MotionMagic;
    elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 20;
    elevatorMotionMagicConfig.MotionMagicAcceleration = 35;
    elevatorMotionMagicConfig.MotionMagicJerk = 70;

    elevatorMotor.getConfigurator().apply(elevatorPIDConfig);

    m_elevatorRequest = new MotionMagicVoltage(0);

    rollerLimitSwitch = rollerMotor.getAnalog(Mode.kAbsolute);

    ntDispTab("Stinger")
      .add("Current Elevator Pos", this::getExtension)
      .add("Current Wrist Angle", this::getAngle);
  }

  @Override
  public void periodic() {
  }

  public void setAngle(double angle) {
    wristMotor.getEncoder().setPosition(angle * StingerConstants.WRIST_GEAR_RATIO);
  }

  public void setElevatorPos(double targetPosRotation) {
    if (targetPosRotation < 0.4) return;
    elevatorMotor.setControl(m_elevatorRequest.withPosition(targetPosRotation));
  }

  public void setElevatorVelocity(double targetVelocity) {
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
    return wristMotor.getEncoder().getPosition() / StingerConstants.WRIST_GEAR_RATIO;
  }

  public double getExtension() {
    return elevatorMotor.getRotorPosition().getValueAsDouble();
  }

  public double getFeedRate() {
    return rollerMotor.getEncoder().getVelocity();
  }

  public boolean isClearOfShooter() {
    return getAngle() > StingerConstants.TRAP_CLEARANCE_ANGLE;
  }

  public boolean stingerRollerHasNote() {
    return rollerLimitSwitch.getVoltage() > 1.0;
  }

}
