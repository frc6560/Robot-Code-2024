// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands.Autos;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShooter extends Command {
  final Shooter shooter;
  final Limelight limelight;
  final Drivetrain drivetrain;
  final boolean shootWhenReady;

  boolean isShooting = false;
  final double shootDuration = 30;
  double shootCounter = 0;
  
  /** Creates a new AutoShooterHead. */
  public AutoShooter(Shooter shooter, Limelight limelight, Drivetrain drivetrain, boolean shootWhenReady) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.shootWhenReady = shootWhenReady;

    addRequirements(shooter, drivetrain);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(shooter.getTransferSensorTriggered() || isShooting){
      shooter.setArcPosition(Constants.SHOOTER_SUBWOOFER_POSITION);
      shooter.setRPM(Constants.SHOOTER_SUBWOOFER_RPM);

      if(shootWhenReady)
        drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
                getRotation(),
                drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
      

      if(shootWhenReady && (shooter.readyToShoot() || isShooting)){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);

        isShooting = true;
        shootCounter ++;
      }
    } else {
      shooter.setRPM(0.0);
      shooter.setTransfer(0.0);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
    }

    if(shootDuration >= shootCounter){
      isShooting = false;
      shootCounter = 0;
    }
  }

  private double getRotation(){
    double limelightInput = -limelight.getHorizontalAngle();
    double llDeadband = 2; // in degrees
    double rotateSpeed = 0.3; // multiplyer for max speed

    double speed = 0;
    double p = 0.25;

    if(Math.abs(limelightInput) < llDeadband){
        limelightInput = 0;
    }

    speed = -limelightInput * p * rotateSpeed;

    return speed;
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0.0);
    shooter.setTransfer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
