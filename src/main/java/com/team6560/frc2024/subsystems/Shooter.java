// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  final Limelight limelight;
  final Trap trap;

  final TalonFX arcMotor;
  final TalonFX shooterMotorRight;
  final TalonFX shooterMotorLeft;

  final CANSparkMax feedMotor;

  // final ColorSensorV3 colorSensor;

  double targetRPM = 0;
  double targetAngle = 0;

  public Shooter(Limelight limelight, Trap trap) {
    this.limelight = limelight;
    this.trap = trap;

    arcMotor = new TalonFX(Constants.ARC_MOTOR);
    shooterMotorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
    shooterMotorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);

    feedMotor = new CANSparkMax(Constants.SHOOTER_FEED_MOTOR, MotorType.kBrushless);

    // colorSensor = new ColorSensorV3(Constants.SHOOTER_COLOR_SENSOR_ID);


    setupMotors();



    ntDispTab("Shooter")
    .add("Shooter RPM", this::getShooterRPM)
    .add("Current Draw Shooter", this::getCurrentDraw)
    .add("Vertical Angle Shooter", this::getShooterVerticalAngle)
    .add("Feeder Proximity Sensor", this::getTransferSensorTriggered);
  }

  private void setupMotors(){
    arcMotor.setNeutralMode(NeutralModeValue.Coast);
    arcMotor.setInverted(false);
    arcMotor.setPosition(0.0);

    shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    shooterMotorLeft.setInverted(false);

    shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);
    shooterMotorRight.setInverted(true);

    feedMotor.restoreFactoryDefaults();
    feedMotor.setInverted(false);
  }

  @Override
  public void periodic() {
  }
  public void setRPM(double speed){
    targetRPM = speed;
    speed /=  Constants.FALCON_MAX_RPM;
    
    shooterMotorRight.set(speed); // TODO: Fix this to pid controlled
    shooterMotorLeft.set(speed);
  }

  public void setVerticalAngle(double angle){
    angle = Math.min(Math.max(angle, Constants.SHOOTER_MIN_POS), Constants.SHOOTER_MAX_POS);
    
    targetAngle = angle;
    
    arcMotor.setPosition(angle * Constants.SHOOTER_ARC_GEAR_RATIO);
  }

  public void setTransfer(double output){
    feedMotor.set(output);
  }
 


  public boolean readyToShoot(){
    return (
      Math.abs(getShooterRPM() - targetRPM) < Constants.SHOOTER_ACCEPTABLE_RPM_DIFF &&
      Math.abs(getShooterVerticalAngle() - targetAngle) < Constants.SHOOTER_ACCEPTABLE_ARC_DIFF &&
      (limelight.hasTarget() && Math.abs(limelight.getHorizontalAngle()) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF ) && 
      trap.isClearOfShooter()
    );
  }


  public boolean getTransferSensorTriggered(){
    // return colorSensor.getProximity() > Constants.SENSOR_TRIGGER_PROXIMITY_VALUE;
    return false;
  }

  // public boolean canIntakeNote(){
  //   return (
  //     Math.abs(getShooterVerticalAngle() - Constants.SHOOTER_INTAKE_POS) < Constants.SHOOTER_ACCEPTABLE_ARC_DIFF &&
  //     !getTransferSensorTriggered()
  //   );
  // }

  public double getShooterRPM(){
    return shooterMotorLeft.getVelocity().getValueAsDouble();
  }

  public double getCurrentDraw(){
    return shooterMotorLeft.getTorqueCurrent().getValueAsDouble();
  }

  public double getShooterVerticalAngle(){
    return arcMotor.getPosition().getValueAsDouble();
  }
}
