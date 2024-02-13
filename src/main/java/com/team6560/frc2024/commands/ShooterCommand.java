// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import java.util.HashMap;
import java.util.Map;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
  final Shooter shooter;
  final ManualControls controls;
  final Limelight limelight;
  final Trap trap;

  class AimTrajectory{
    double rpm;
    double angle;

    public AimTrajectory(double rpm, double angle){
      this.rpm = rpm;
      this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
    public double getRpm() {
        return rpm;
    }
  }

  Map<Double,AimTrajectory> aimMap = new HashMap<>();


  public ShooterCommand(Shooter shooter, Trap trap, Limelight limelight, ManualControls controls) {
    this.shooter = shooter;
    this.trap = trap;
    this.limelight = limelight;
    this.controls = controls;

    addRequirements(shooter);

    aimMap.put(0.0, new AimTrajectory(3000, 30));
    aimMap.put(10.0, new AimTrajectory(6000, 60));
  }

  @Override
  public void initialize() {
    shooter.setRPM(0.0);
  }

  @Override
  public void execute() {
    if (controls.getAim() || controls.getSafeAim()) {
      double dist = limelight.getDistance();


      if (controls.getSafeAim()){
        shooter.setRPM(Constants.SHOOTER_SUB_RPM);
        shooter.setVerticalAngle(Constants.SHOOTER_SUB_ANGLE);
      } else {
        shooter.setRPM(getShootRPM(dist));
        shooter.setVerticalAngle(getShootAngle(dist));
      }
      
      if (controls.getShoot() && shooter.readyToShoot()){
        shooter.setTransfer(Constants.TRANSFER_FEED_OUTPUT);
      }

    } else if (controls.getRunIntake()){
      shooter.setVerticalAngle(Constants.SHOOTER_GROUND_INTAKE_ANGLE);
      shooter.setRPM(0.0);

      if(!shooter.getTransferSensorTriggered()){

        shooter.setTransfer(Constants.TRANSFER_INTAKE_OUTPUT);
      } else {
        shooter.setTransfer(0.0);
      }
    } else {
      shooter.setRPM(0.0);
    }

  }

  private double[] findClosest(double dist){
    double lower = 0;
    double upper = 0;

    for (Map.Entry<Double,AimTrajectory> e : aimMap.entrySet()){
      if(e.getKey() < dist){
        lower = e.getKey();
      
      } else {
        upper = e.getKey();

        return new double[]{lower, upper};
      }
    }

    return new double[]{lower, upper};
  }

  public double getShootRPM(double dist){
    double lowerDist = findClosest(dist)[0];
    double upperDist = findClosest(dist)[1];
    
    double lowerRPM = aimMap.get(lowerDist).getRpm();
    double upperRPM = aimMap.get(upperDist).getRpm();
    
    
    return (dist - lowerDist) / (upperDist - lowerDist) * (upperRPM - lowerRPM) + lowerRPM;
  }

  public double getShootAngle(double dist){
    double lowerDist = findClosest(dist)[0];
    double upperDist = findClosest(dist)[1];
    
    double lowerAngle = aimMap.get(lowerDist).getAngle();
    double upperAngle= aimMap.get(upperDist).getAngle();
    
    
    return (dist - lowerDist) / (upperDist - lowerDist) * (upperAngle - lowerAngle) + lowerAngle;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
