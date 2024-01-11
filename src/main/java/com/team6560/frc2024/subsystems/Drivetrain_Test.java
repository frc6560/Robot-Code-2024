// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.team6560.frc2024.subsystems.swerve.SwerveModuleConstants;
import com.team6560.frc2024.subsystems.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.team6560.frc2024.subsystems.swerve.SwerveDrivetrain;
import com.team6560.frc2024.subsystems.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain_Test extends SubsystemBase {
  /** Creates a new Drivetrain_Test. */
  public Drivetrain_Test() {
    SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants();
    SwerveModuleConstants moduleConstants = new SwerveModuleConstants()
      .withCANcoderId(0)
      .withCANcoderOffset(0)
      .withCouplingGearRatio(0)
      .withDriveMotorGearRatio(0)
      .withDriveMotorId(0)
      .withWheelRadius(0)
      .withLocationX(0)
      .withLocationY(0)
      .withSteerMotorGearRatio(0.0)
      .withSteerMotorId(0);
    SwerveDrivetrain test = new SwerveDrivetrain(drivetrainConstants, moduleConstants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
