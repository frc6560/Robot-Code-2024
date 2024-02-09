// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2024.Constants.ShooterConstants;
import com.team6560.frc2024.utility.NetworkTable.NtValueDisplay;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX m_shooterLeft;
  private final TalonFX m_shooterRight;
  // private final VelocityVoltage m_shooterRequest;

  private final TalonFX m_arc;
  
  private double targetRPM;
  private double targetAngle;

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Shooter");

  private NetworkTableEntry ntRPM;
  private NetworkTableEntry ntAngle;

  // private final double arcTurnSpeed = 0.5;
  

  public Shooter() {
    targetRPM = 0;
    targetAngle = 0;


    //Shooter Motor Config
    m_shooterLeft = new TalonFX(ShooterConstants.SHOOTER_LEFT_ID);
    m_shooterRight = new TalonFX(ShooterConstants.SHOOTER_RIGHT_ID);

    m_shooterLeft.getConfigurator().apply(new TalonFXConfiguration());
    m_shooterRight.getConfigurator().apply(new TalonFXConfiguration());

    var shooterPIDConfig = new Slot0Configs();
    shooterPIDConfig.kS = 0.04;
    shooterPIDConfig.kV = 0.1;
    shooterPIDConfig.kP = 0.1;
    shooterPIDConfig.kI = 0;
    shooterPIDConfig.kD = 0;
    
    var shooterGearRatio = new FeedbackConfigs();
    shooterGearRatio.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;

    m_shooterLeft.getConfigurator().apply(shooterPIDConfig);
    m_shooterRight.getConfigurator().apply(shooterPIDConfig);

    m_shooterLeft.setInverted(false);
    m_shooterRight.setInverted(true);

    //Arc Motor Config
    m_arc = new TalonFX(ShooterConstants.ARC_MOTOR_ID);
    m_arc.getConfigurator().apply(new TalonFXConfiguration());

    var arcPIDConfig = new Slot0Configs();
    arcPIDConfig.kS = 0;
    arcPIDConfig.kV = 0;
    arcPIDConfig.kP = 0.1;
    arcPIDConfig.kD = 0;
    arcPIDConfig.kI = 0.0001;

    

    ntDispTab("Shooter")
      .add("Current RPM", this::getShooterRPM)
      .add("Target RPM", () -> targetRPM)
      .add("Current Arc Angle", this::getArcAngle)
      .add("Target Arc Angle", () -> targetAngle);
    
    SmartDashboard.putNumber("Shooter RPM", 0.0);
    SmartDashboard.putNumber("Shooter Angle", 0.0);
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooterLeft.set(targetRPM / ShooterConstants.RPM_PER_FALCON_UNIT);
    m_shooterRight.set(targetRPM / ShooterConstants.RPM_PER_FALCON_UNIT);
  }

  public boolean isReadyAutoAim() {
    if (getRPMDifference() < ShooterConstants.ACCEPTABLE_RPM_DIFF && getAngleDifference() < ShooterConstants.ACCEPTABLE_ANGLE_DIFF) {
      return true;
    }   
    return false;
  }

  public boolean isReadyManualAim() {
    if (getRPMDifference() < ShooterConstants.ACCEPTABLE_RPM_DIFF) {
      return true;
    }   
    return false;
  }

  public void setManualAim(double speed) {
    m_arc.set(speed);
  }

  public double getArcAngle() {
    return m_arc.getPosition().getValue();
  }

  public double getShooterRPM() {
    return m_shooterLeft.getVelocity().getValue();
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public void setTargetRPM (double input) {
    targetRPM = input;
  }

  public void setTargetAngle (double angle) {
    targetAngle = angle;
  }

  public double getRPMDifference() {
    return Math.abs(targetRPM - getShooterRPM());
  }

  public double getAngleDifference() {
    return Math.abs(targetAngle - getArcAngle());
  }

  public double getSmartDashboardRPM() {
    return SmartDashboard.getNumber("Shooter RPM", targetRPM);
  }

  public double getSmartDashboardAngle() {
    return SmartDashboard.getNumber("Shooter Angle", targetAngle);
  }

}
