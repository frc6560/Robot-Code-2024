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
  Drivetrain drivetrain;

  boolean isShooting = false;

  double endTimer = 0;
  double endTotalTime = 10;

  double rpm, arc;
  
  boolean shootingSafe;
  
  /** Creates a new AutoShooterHead. */
  public AutoShooter(Shooter shooter, Limelight limelight, double arc, double rpm) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.drivetrain = null;

    this.arc = arc;
    this.rpm = rpm;

    addRequirements(shooter);

    this.shootingSafe = true;
  }

  public AutoShooter(Shooter shooter, Limelight limelight, Drivetrain drivetrain){
    this(shooter, limelight, -1, -1);
    this.drivetrain = drivetrain;
    this.shootingSafe = false;

  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    
    limelight.setPipeline(Constants.isRed() ? 1 : 0);

    
    if(shooter.getTransferSensorTriggered() || isShooting){
      if(arc == -1 && rpm == -1){
        if(limelight.hasTarget()){
          arc = shooter.findClosest(limelight.getVerticalAngle()).getAngle();
          rpm = shooter.findClosest(limelight.getVerticalAngle()).getRpm();
        } else {
          // shootingSafe = true;
          arc = shooter.findClosest(6).getAngle();
          rpm = shooter.findClosest(6).getRpm();

        }
      }

      shooter.setArcPosition(arc);//shooter.findClosest(1.6).getAngle());//Constants.SHOOTER_SUBWOOFER_POSITION);
      shooter.setRPM(rpm);//shooter.findClosest(1.4).getRpm());//Constants.SHOOTER_SUBWOOFER_RPM);

      if(!shootingSafe){
        aimChassis();
      }


      if(shooter.readyToShoot(shootingSafe) || isShooting){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);

        isShooting = true;
      }
    } else {
      shooter.setRPM(0.0);
      shooter.setTransfer(0.0);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
    }

    if(!shooter.getTransferSensorTriggered()){
      endTimer++;
    } 
  }

  private double getRotation(){
    double limelightInput = limelight.hasTarget() ? -limelight.getHorizontalAngle() : 0;
    double rotateSpeed = 0.3; // multiplyer for max speed

    double speed = 0;
    double p = 0.3;

    if(Math.abs(limelightInput) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF/2){
        limelightInput = 0;
    }

    speed = limelightInput * p * rotateSpeed;

    return speed;
}

  private void aimChassis(){
    drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                getRotation(),
                drivetrain.getGyroscopeRotationNoApriltags()));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0.0);
    shooter.setTransfer(0.0);
    isShooting = false;
    endTimer = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endTimer >= endTotalTime;
  }
}
