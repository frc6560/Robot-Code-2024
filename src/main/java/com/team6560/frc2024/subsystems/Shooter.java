// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
  double targetPosition = 0;

  public Shooter(Limelight limelight, Trap trap) {
    this.limelight = limelight;
    this.trap = trap;

    arcMotor = new TalonFX(Constants.ARC_MOTOR);
    shooterMotorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
    shooterMotorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);

    feedMotor = new CANSparkMax(Constants.SHOOTER_FEED_MOTOR, MotorType.kBrushless);

    colorSensor = new ColorSensorV3(Constants.SHOOTER_COLOR_SENSOR_PORT);


    setupMotors();



    ntDispTab("Shooter")
    .add("Shooter RPM", this::getShooterRPM)
    .add("Current Draw Shooter", this::getCurrentDraw)
    .add("Vertical Position Shooter", this::getShooterArcPosition)
    .add("Vertical Angle Shooter", this::getShooterArcAngleDegrees)
    .add("Transfer Triggered", this::getTransferSensorTriggered)
    .add("Feeder Proximity Sensor", ()->colorSensor.getProximity());
  }

  private void setupMotors(){
    arcMotor.setNeutralMode(NeutralModeValue.Brake);
    arcMotor.setInverted(false);
    
    // in init function
    TalonFXConfiguration arcConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    Slot0Configs arcPID = arcConfigs.Slot0;
    arcPID.kS = 0.5; // Add 0.25 V output to overcome static friction // Static Feed Forward
    arcPID.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    arcPID.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    arcPID.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    arcPID.kI = 0; // no output for integrated error
    arcPID.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    MotionMagicConfigs arcMM = arcConfigs.MotionMagic;
    arcMM.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    arcMM.MotionMagicAcceleration = arcMM.MotionMagicCruiseVelocity / 0.5; // Target acceleration of 160 rps/s (0.5 seconds)
    arcMM.MotionMagicJerk = 0;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    arcMotor.getConfigurator().apply(arcConfigs);

    TalonFX[] shooterMotors = new TalonFX[]{shooterMotorLeft,shooterMotorRight};

    for (TalonFX motor: shooterMotors){
      motor.setNeutralMode(NeutralModeValue.Coast);

      // in init function, set slot 0 gains
      var slot0Configs = new Slot0Configs();
      slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0; // no output for integrated error
      slot0Configs.kD = 0; // no output for error derivative

      motor.getConfigurator().apply(slot0Configs);
    }
    
    shooterMotorLeft.setInverted(false);
    shooterMotorRight.setInverted(true);

    feedMotor.restoreFactoryDefaults();
    feedMotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }


  public void setRPM(double speed){
    targetRPM = speed;
    speed = speed / 60;


    final VelocityVoltage m_request = new VelocityVoltage(speed).withSlot(0);

    shooterMotorRight.setControl(m_request);
    shooterMotorLeft.setControl(m_request);
  }

  public void setArcPosition(double position){    
    targetPosition = position;
    
    final MotionMagicVoltage m_request = new MotionMagicVoltage(position);
    arcMotor.setControl(m_request);
  }

  public void setArcOutput(double output){
    arcMotor.set(output);
  }

  public void setTransfer(double output){
    feedMotor.set(output);
  }
 


  public boolean readyToShoot(){
    return ( 
      Math.abs(getShooterRPM() - targetRPM) < Constants.SHOOTER_ACCEPTABLE_RPM_DIFF &&
      Math.abs(getShooterArcPosition() - targetPosition) < Constants.SHOOTER_ACCEPTABLE_ARC_DIFF
      // (limelight.hasTarget() && Math.abs(limelight.getHorizontalAngle()) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF ) && 
      // trap.isClearOfShooter()
    );
  }


  public boolean getTransferSensorTriggered(){
    return colorSensor.getProximity() > Constants.SENSOR_TRIGGER_PROXIMITY_VALUE;
  }


  public double getShooterRPM(){
    return shooterMotorLeft.getRotorVelocity().getValueAsDouble() * 60;
  }

  public double getCurrentDraw(){
    return shooterMotorLeft.getTorqueCurrent().getValueAsDouble();
  }

  public double getShooterArcPosition(){
    return arcMotor.getRotorPosition().getValueAsDouble();
  }
  public double getShooterArcAngleDegrees(){
    return getShooterArcPosition() / Constants.SHOOTER_ARC_GEAR_RATIO * 360;
  }
}
