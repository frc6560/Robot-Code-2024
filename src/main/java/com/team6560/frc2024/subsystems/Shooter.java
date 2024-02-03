// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2024.Constants.ShooterConstants;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX m_shooterLeft;
  private final TalonFX m_shooterRight;
  private final VelocityVoltage m_shooterRequest;

  private final TalonFX m_arc;
  private final PositionVoltage m_arcRequest;

  // private final CANSparkMax m_transfer;
  
  private double targetRPM;
  private double targetAngle;

  private NetworkTable ntTable;
  

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
    
    var shooterGearRatio = new FeedbackConfigs();
    shooterGearRatio.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;

    m_shooterLeft.getConfigurator().apply(shooterPIDConfig, 0);
    m_shooterRight.getConfigurator().apply(shooterPIDConfig, 0);

    m_shooterLeft.getConfigurator().apply(shooterGearRatio, 0);//not sure if this is the correct way to set gear ratio for the motor in order to get accurate angle position and velocity of the shooter
    m_shooterRight.getConfigurator().apply(shooterGearRatio, 0); //not sure if this is the correct way to set gear ratio for the motor in order to get accurate angle position and velocity of the shooter

    m_shooterLeft.setInverted(false);
    m_shooterRight.setInverted(true);

    m_shooterRequest = new VelocityVoltage(0).withSlot(0);

    //Arc Motor Config
    m_arc = new TalonFX(ShooterConstants.ARC_MOTOR_ID);
    m_arc.getConfigurator().apply(new TalonFXConfiguration());

    var arcGearbox = new FeedbackConfigs();
    arcGearbox.SensorToMechanismRatio = ShooterConstants.ARC_GEAR_RATIO;

    var arcPIDConfig = new Slot1Configs();
    arcPIDConfig.kP = 0.1;
    arcPIDConfig.kD = 0;
    arcPIDConfig.kI = 0.0001;;

    m_arc.getConfigurator().apply(arcPIDConfig, 0);
    m_arc.getConfigurator().apply(arcGearbox, 0); //not sure if this is the correct way to set gear ratio for the motor in order to get accurate angle position and velocity of the arc

    m_arcRequest = new PositionVoltage(0).withSlot(1);

    //Transfer Motor Config
    // m_transfer = new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
    // m_transfer.getEncoder().setPosition(10);

    ntTable = NetworkTableInstance.getDefault().getTable("Shooter");

    ntDispTab("Shooter")
      .add("Current RPM", this::getShooterRPM)
      .add("Target RPM", () -> targetRPM)
      
      .add("Current Arc Angle", this::getArcAngle)
      .add("Target Arc Angle", () -> targetAngle);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooterLeft.setControl(m_shooterRequest.withVelocity(targetRPM / ShooterConstants.RPM_PER_FALCON_UNIT).withFeedForward(0.1));
    m_shooterRight.setControl(m_shooterRequest.withVelocity(targetRPM / ShooterConstants.RPM_PER_FALCON_UNIT).withFeedForward(0.1));

    double angleDiff = targetAngle - getArcAngle();

    if (Math.abs(angleDiff) > ShooterConstants.ACCEPTABLE_ANGLE_DIFF) {
      m_arc.setControl(m_arcRequest.withPosition(targetAngle).withFeedForward(0.1));
    }
  }

  public double getArcAngle() {
    return m_arc.getPosition().getValueAsDouble();
  }

  public double getShooterRPM() {
    return m_shooterLeft.getVelocity().getValueAsDouble();
  }

  public void setTargetRPM (double input) {
    targetRPM = input;
  }

  public void setTargetAngle (double angle) {
    targetAngle = angle;
  }


}
