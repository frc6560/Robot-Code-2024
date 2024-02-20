// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
public class ShooterCommand extends Command {
  final Shooter shooter;
  final ManualControls controls;
  final Limelight limelight;
  final Trap trap;
  final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Shooter");
  final NetworkTableEntry manualShooter;
  final NetworkTableEntry shooterSetAngle;
  final NetworkTableEntry shooterSetRPM;

  class AimTrajectory implements Interpolatable<AimTrajectory>{
    double rpm;
    double angle;

    double distance;

    public AimTrajectory(double distance, double rpm, double angle){
      this.rpm = rpm;
      this.angle = angle;
      this.distance = distance;
    }

    public double getDistance(){
      return distance;
    }
    public double getAngle() {
        return angle;
    }
    public double getRpm() {
        return rpm;
    }

    @Override
    public AimTrajectory interpolate(AimTrajectory endValue, double t) {
      double angle = getAngle() + (endValue.getAngle() - getAngle()) * t;
      double rpm = getRpm() + (endValue.getRpm() - getRpm()) * t;
      double distance = getDistance() + (endValue.getDistance() - getDistance()) * t;

      return new AimTrajectory(distance, rpm, angle);
    }

    public String toString(){
      return "Distance: " + getDistance() + ", Angle: " + getAngle() + ", RPM: " + getRpm();
    }
  }


  ArrayList<AimTrajectory> aimMap = new ArrayList<>();


  public ShooterCommand(Shooter shooter, Trap trap, Limelight limelight, ManualControls controls) {
    this.shooter = shooter;
    this.trap = trap;
    this.limelight = limelight;
    this.controls = controls;

    addRequirements(shooter);

    aimMap.add(new AimTrajectory(-100.0, 6100 , 17));

    aimMap.add(new AimTrajectory(-3.19, 6100 , 17));
    aimMap.add(new AimTrajectory(-1.93, 6100 , 16));
    aimMap.add(new AimTrajectory(-0.345, 6100 , 14));
    aimMap.add(new AimTrajectory(1.29, 6100 , 16.5));
    aimMap.add(new AimTrajectory(2.73, 6000 , 19));
    aimMap.add(new AimTrajectory(4.42, 6000 , 21));
    aimMap.add(new AimTrajectory(6.76, 6000 , 25));
    aimMap.add(new AimTrajectory(9.53, 6000 , 30));
    aimMap.add(new AimTrajectory(13.47, 6000 , 35));
    aimMap.add(new AimTrajectory(18.70, 5800 , 40));
    aimMap.add(new AimTrajectory(20.0, 5500 , 45));
    
    aimMap.add(new AimTrajectory(100.0, 5500 , 45));

    manualShooter = ntTable.getEntry("Manual Arc");
    manualShooter.setBoolean(false);

    shooterSetAngle = ntTable.getEntry("Go to Position");
    shooterSetAngle.setDouble(0.0);
    shooterSetRPM = ntTable.getEntry("Go to rpm");
    shooterSetRPM.setDouble(0.0);
  }

  @Override
  public void initialize() {
    shooter.setRPM(0.0);
  }

  @Override
  public void execute() {

    if(controls.getReverseTransfer()){
      shooter.setRPM(0);
      shooter.setTransfer(-0.2);
    } else if (controls.getRunIntake() && !shooter.getTransferSensorTriggered()){
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
      shooter.setRPM(0.0);
      
      if(!shooter.getTransferSensorTriggered()){
        shooter.setTransfer(Constants.TRANSFER_INTAKE_RATE);

      } else {
        shooter.setTransfer(0);
      }

    
    } else if (controls.getSafeAim() || controls.getAim()){
      double dist = limelight.getVerticalAngle();

      if(controls.getSafeAim()){
        shooter.setArcPosition(Constants.SHOOTER_SUBWOOFER_POSITION);
        shooter.setRPM(Constants.SHOOTER_SUBWOOFER_RPM);
        // shooter.setArcPosition(shooterSetAngle.getDouble(0.0));
        // shooter.setRPM(shooterSetRPM.getDouble(0.0));
      } else {
        shooter.setArcPosition(findClosest(dist).getAngle());
        shooter.setRPM(findClosest(dist).getRpm());
      }

      if (controls.getShoot() && shooter.readyToShoot()){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);
      } else {
        shooter.setTransfer(0);
      }




    } else if (controls.getTrapIntake()) {
      shooter.setArcPosition(Constants.SHOOTER_WALL_INTAKE_POSITION);
    
    }else if (controls.getTrapTransferIn()){
      shooter.setRPM(Constants.SHOOTER_TRANSFER_OUT_RPM);

      if(controls.getShoot()){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);
      } else {
        shooter.setTransfer(0.0);
      }

    } else if (controls.getTrapTransferOut()){
      shooter.setRPM(Constants.SHOOTER_TRANSFER_IN_RPM);
      
      if(!shooter.getTransferSensorTriggered()){
        shooter.setTransfer(-Constants.TRANSFER_INTAKE_RATE);
      } else {
        shooter.setTransfer(0.0);
      }

    } else if(controls.getTrapPlace()){
      shooter.setArcPosition(Constants.SHOOTER_AMP_ARC_POS);
    
    }else {
      shooter.setRPM(0.0);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
      shooter.setTransfer(0.0);
    }

  }

  private AimTrajectory findClosest(double dist){
    AimTrajectory lower = new AimTrajectory(0, Constants.SHOOTER_SUBWOOFER_RPM, Constants.SHOOTER_SUBWOOFER_POSITION);
    AimTrajectory upper = new AimTrajectory(0, Constants.SHOOTER_SUBWOOFER_RPM, Constants.SHOOTER_SUBWOOFER_POSITION);
    double t;

    for(AimTrajectory trajectory : aimMap){
      // System.out.println(trajectory);
      if(trajectory.getDistance() < dist){
        lower = trajectory;
      } else {
        upper = trajectory;
        break;
      }
    }

    t = (dist - lower.getDistance()) / (upper.getDistance() - lower.getDistance());

    AimTrajectory newTrajectory = lower.interpolate(upper, t);
    // System.out.println("new traj = " + newTrajectory);
    return newTrajectory;
  }

  // public double getShootRPM(double dist){
  //   if(dist == 0) return 0;

  //   double lowerDist = findClosest(dist)[0];
  //   double upperDist = findClosest(dist)[1];
    
  //   double lowerRPM = aimMap.get(lowerDist).getRpm();
  //   double upperRPM = aimMap.get(upperDist).getRpm();
    
    
  //   return (dist - lowerDist) / (upperDist - lowerDist) * (upperRPM - lowerRPM) + lowerRPM;
  // }

  // public double getShootAngle(double dist){
  //   if(dist == 0) return 0;
    
  //   Double lowerDist = findClosest(dist)[0];
  //   Double upperDist = findClosest(dist)[1];

  //   System.out.println("lower: " + lowerDist + " upper " + upperDist);
    
  //   double lowerAngle = aimMap.get(lowerDist).getAngle();
  //   double upperAngle= aimMap.get(upperDist).getAngle();
    
    
  //   return (dist - lowerDist) / (upperDist - lowerDist) * (upperAngle - lowerAngle) + lowerAngle;
  // }

  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
