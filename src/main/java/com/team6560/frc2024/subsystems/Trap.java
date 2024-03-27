// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.team6560.frc2024.Constants;

import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trap extends SubsystemBase {
  
  final CANSparkMax wristMotor;
  final TalonFX extentionMotor;

  final CANSparkMax feedMotor;

  final SparkLimitSwitch limitSwitch;

  double targetAngle = 0;
  double targetExtention = 0;

  public Trap() {
    this.wristMotor = new CANSparkMax(Constants.TRAP_WRIST_MOTOR, MotorType.kBrushless);
    this.extentionMotor = new TalonFX(Constants.TRAP_ELEVATOR_MOTOR);
    this.feedMotor = new CANSparkMax(Constants.TRAP_FEED_MOTOR, MotorType.kBrushless);

    limitSwitch = feedMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    limitSwitch.enableLimitSwitch(false);

    setupMotors();

    ntDispTab("Trap")
      .add("Elevator Position", this::getExtention)
      .add("Wrist Angle", this::getAngle)
      .add("Wrist Target Angle", ()->(targetAngle))
      .add("Wrist Target Extention", ()->(targetExtention))
      .add("Feed speed Stinger", this::getFeedRate)
      .add("Trap Sensor Triggered", this::getSensorTriggered);
  }

  private void setupMotors(){
    wristMotor.restoreFactoryDefaults();
    wristMotor.setInverted(true);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.getEncoder().setPosition(0.0);
    
    
    wristMotor.getPIDController().setFF(0.0);
    wristMotor.getPIDController().setP(0.5);
    wristMotor.getPIDController().setI(0);
    wristMotor.getPIDController().setD(0);

    
    extentionMotor.setInverted(true);
    extentionMotor.setNeutralMode(NeutralModeValue.Brake);

    

    
    // in init function
    TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    Slot0Configs elevatorPID = elevatorConfigs.Slot0;
    // elevatorPID.kS = 0.5; // Add 0.25 V output to overcome static friction // Static Feed Forward
    // elevatorPID.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // elevatorPID.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorPID.kP = 2.4; // A position error of 2.5 rotations results in 12 V output
    elevatorPID.kI = 0; // no output for integrated error
    // elevatorPID.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    HardwareLimitSwitchConfigs elevatorSwitches = new HardwareLimitSwitchConfigs();
    elevatorSwitches.ReverseLimitAutosetPositionEnable = true;
    elevatorSwitches.ReverseLimitAutosetPositionValue = 0.0;
elevatorSwitches.ForwardLimitAutosetPositionEnable = true;
    elevatorSwitches.ForwardLimitAutosetPositionValue = 8.65;

    // set Motion Magic settings
    MotionMagicConfigs elevatorMM = elevatorConfigs.MotionMagic;
    elevatorMM.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    elevatorMM.MotionMagicAcceleration = elevatorMM.MotionMagicCruiseVelocity / 0.5; // Target acceleration of 160 rps/s (0.5 seconds)
    elevatorMM.MotionMagicJerk = 0;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    extentionMotor.getConfigurator().apply(elevatorConfigs.withHardwareLimitSwitch(elevatorSwitches));
    // extentionMotor


    feedMotor.restoreFactoryDefaults();
    feedMotor.setInverted(false);
    feedMotor.setIdleMode(IdleMode.kBrake);
    limitSwitch.enableLimitSwitch(false);


    wristMotor.getEncoder().setPosition(0.0);
    extentionMotor.setPosition(0.0);
  }

  @Override
  public void periodic() {
  }

  
  public void setAngle(double pos){
    targetAngle = pos;
    wristMotor.getPIDController().setReference(pos, ControlType.kPosition);
  }

  public void setExtention(double pos){
    targetExtention = pos;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(pos);
    extentionMotor.setControl(m_request);
  }

  public void setFeed(double output){
    feedMotor.set(output);
  }


  public double getAngle(){
    return wristMotor.getEncoder().getPosition() / Constants.TRAP_WRIST_GEAR_RATIO;
  }

  public double getExtention(){
    return extentionMotor.getRotorPosition().getValueAsDouble();//getEncoder().getPosition() / Constants.TRAP_ELEVATOR_GEAR_RATIO;
  }

  public double getFeedRate(){
    return feedMotor.getEncoder().getVelocity();
  }

  public boolean getSensorTriggered(){
    return limitSwitch.isPressed();
  }

  public boolean isClearOfShooter(){
    return getAngle() < Constants.TRAP_CLEARANCE_ANGLE;
  }

  public boolean isAtTargetAngle() {
      return Math.abs(wristMotor.getEncoder().getPosition() - targetAngle) < Constants.TRAP_ACCEPTABLE_ANGLE_DIFF;
  }

  public boolean isAtTargetExtention() {
      return Math.abs(extentionMotor.getRotorPosition().getValueAsDouble() - targetExtention) < Constants.TRAP_ACCEPTABLE_EXTENTION_DIFF;
  }
}
