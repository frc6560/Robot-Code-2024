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

import com.team6560.frc2024.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX m_shooterLeft;
  private final TalonFX m_shooterRight;

  private final CANSparkMax m_angleMotor;
  
  private double targetRPM;
  private double targetAngle;

  public Shooter() {
    targetRPM = 0;
    targetAngle = 0;


    //Shooter Motor Config
    m_shooterLeft = new TalonFX(ShooterConstants.SHOOTER_LEFT_ID);
    m_shooterRight = new TalonFX(ShooterConstants.SHOOTER_RIGHT_ID);

    m_shooterLeft.getConfigurator().apply(new TalonFXConfiguration());
    m_shooterRight.getConfigurator().apply(new TalonFXConfiguration());

    var shooterPIDConfig = new Slot0Configs();
    shooterPIDConfig.kV = 0.1;
    shooterPIDConfig.kP = 0.1;
    shooterPIDConfig.kI = 0.0001;
    shooterPIDConfig.kD = 0.0;

    m_shooterLeft.getConfigurator().apply(shooterPIDConfig, 0);
    m_shooterRight.getConfigurator().apply(shooterPIDConfig, 0);

    //Angle Motor Config
    m_angleMotor = new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
    m_angleMotor.getEncoder().setPosition(10);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
