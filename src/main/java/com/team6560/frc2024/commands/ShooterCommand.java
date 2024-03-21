// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

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

  public ShooterCommand(Shooter shooter, Trap trap, Limelight limelight, ManualControls controls) {
    this.shooter = shooter;
    this.trap = trap;
    this.limelight = limelight;
    this.controls = controls;

    addRequirements(shooter);


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
    limelight.setPipeline(Constants.isRed() ? 1 : 0);

    if(controls.getReverseTransfer()){
      shooter.setRPM(-500);
      shooter.setTransfer(-0.2);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
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
        // shooter.setArcPosition(shooterSetAngle.getDouble(45));
        // shooter.setRPM(shooterSetRPM.getDouble(5500));
      } else {
        shooter.setArcPosition(shooter.findClosest(dist).getAngle());
        shooter.setRPM(shooter.findClosest(dist).getRpm());
      }

      if (controls.getShoot() && shooter.readyToShoot(controls.getSafeAim())){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);
      } else {
        shooter.setTransfer(0);
      }



    } else if (controls.getTrapPlace()) {
      shooter.setArcPosition(Constants.SHOOTER_TRAP_ARC_POS);

    } else if (controls.getTrapIntake() && !trap.getSensorTriggered()) {
      shooter.setArcPosition(Constants.SHOOTER_WALL_INTAKE_POSITION);
    
    }else if (controls.getTrapTransferIn()){
      if(!trap.getSensorTriggered()){
        shooter.setRPM(Constants.SHOOTER_TRANSFER_OUT_RPM);
      } else {
        shooter.setRPM(0.0);
      }

      if(controls.getShoot() && trap.isAtTargetAngle() && trap.isAtTargetExtention() && !trap.getSensorTriggered()){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);
      } else {
        shooter.setTransfer(0.0);
      }

    } else if(controls.getAmpPlace()){
      shooter.setArcPosition(Constants.SHOOTER_AMP_ARC_POS);
    
    } else if (controls.getTrapPlaceOver()) {
      if (trap.isAtTargetExtention()) {
        shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
      }

    } else {
      shooter.setRPM(0.0);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
      shooter.setTransfer(0.0);
    }

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
