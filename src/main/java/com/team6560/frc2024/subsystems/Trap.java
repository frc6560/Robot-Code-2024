// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;

import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trap extends SubsystemBase {
  
  final CANSparkMax wristMotor;
  final TalonFX trapElevator;

  final CANSparkMax feedMotor;

  final DigitalInput limitSwitch = new DigitalInput(Constants.TRAP_LIMIT_SWITCH_ID);

  public Trap() {
    this.wristMotor = new CANSparkMax(Constants.TRAP_WRIST_MOTOR, MotorType.kBrushless);
    this.trapElevator = new TalonFX(Constants.TRAP_ELEVATOR_MOTOR);
    this.feedMotor = new CANSparkMax(Constants.TRAP_FEED_MOTOR, MotorType.kBrushless);

    setupMotors();

    ntDispTab("Trap")
      .add("Elevator Position", this::getExtention)
      .add("Wrist Angle", this::getAngle)
      .add("Feed speed Stinger", this::getFeedRate)
      .add("Trap Sensor Triggered", this::getSensorTriggered);
  }

  private void setupMotors(){
    wristMotor.restoreFactoryDefaults();
    wristMotor.setInverted(true);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.getPIDController().setP(0);



    
    trapElevator.setInverted(true);
    trapElevator.setNeutralMode(NeutralModeValue.Brake);

    // trapElevator





    feedMotor.restoreFactoryDefaults();
    feedMotor.setInverted(false);
    feedMotor.setIdleMode(IdleMode.kBrake);

    feedMotor.getPIDController().setP(0);


    wristMotor.getEncoder().setPosition(0.0);
    trapElevator.setPosition(0.0);
  }

  @Override
  public void periodic() {
  }

  
  public void setAngle(double angle){
    // wristMotor.getEncoder().setPosition(angle * Constants.TRAP_WRIST_GEAR_RATIO);
    wristMotor.set(angle);
  }

  public void setExtention(double angle){
    // trapElevator.setPosition(angle, 20);
    trapElevator.set(angle);
  }

  public void setFeed(double output){
    feedMotor.set(output);
  }


  public double getAngle(){
    return wristMotor.getEncoder().getPosition() / Constants.TRAP_WRIST_GEAR_RATIO;
  }

  public double getExtention(){
    return trapElevator.getPosition().getValueAsDouble();//getEncoder().getPosition() / Constants.TRAP_ELEVATOR_GEAR_RATIO;
  }

  public double getFeedRate(){
    return feedMotor.getEncoder().getVelocity();
  }

  public boolean getSensorTriggered(){
    return limitSwitch.get();
  }

  public boolean isClearOfShooter(){
    return getAngle() < Constants.TRAP_CLEARANCE_ANGLE;
  }
}
