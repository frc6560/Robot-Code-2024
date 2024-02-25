// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Stinger;
import com.team6560.frc2024.subsystems.Transfer;
import com.team6560.frc2024.Constants.StingerConfigs;
import com.team6560.frc2024.Constants.StingerConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class StingerCommand extends Command {
  /** Creates a new StingerCommand. */

  public static interface Controls {
    boolean getHumanStationIntake();

    boolean getShooterStingerTransfer();
  }

  private final Stinger stinger;
  private final Shooter shooter;
  private final Transfer transfer;
  private final Controls controls;


  private boolean isDoneTransfer = true, shooterStingerAligned = false, correctShooterRpm = false, maxTransferSensorReached = false, transferHasNote = false;
  private boolean stingerToIntakePos = false;



  public StingerCommand(Stinger stinger, Shooter shooter, Transfer transfer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stinger = stinger;
    this.shooter = shooter;
    this.transfer = transfer;
    this.controls = controls;

    addRequirements(stinger); 
    
  }

  public void setElevatorPosPresets(StingerConfigs pos) {
    stinger.setElevatorPos(pos.getElevatorPos());
  }

  public void setWristPosPreset(StingerConfigs pos) {
    stinger.setAngle(pos.getStingerAngle());
  }

  public void setBothPosPresets(StingerConfigs pos) {
    stinger.setElevatorPos(pos.getElevatorPos());
    stinger.setAngle(pos.getStingerAngle());
  }

  public boolean isAutoTransferReady() {
    return (!transfer.isInProximity() && stinger.stingerRollerHasNote());
  }

  public void autoIntakeHumanStation() {
    if (isAutoTransferReady() || transfer.isInProximity()) 
      return;
    else {
      if (!stingerToIntakePos)
        setElevatorPosPresets(StingerConfigs.HUMAN_STATION_INTAKE);
      
        // if stingerToIntakePos is false but at the right position, set stingerToIntakePos to true
      if (!stingerToIntakePos 
          && Math.abs(stinger.getAngle() - StingerConfigs.HUMAN_STATION_INTAKE.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
          && Math.abs(stinger.getExtension() - StingerConfigs.HUMAN_STATION_INTAKE.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF) {
          
          stingerToIntakePos = true;
        }

      if (stingerToIntakePos) {
        stinger.setRoller(1); //placeholder value
      }

      if (stinger.stingerRollerHasNote()) {
        stinger.setRoller(0);

        stingerToIntakePos = false;
      }
    }
  }

  public void autoTransferToShooter() { 
    if (transfer.isInProximity() && isDoneTransfer) return;
    
    else {
      isDoneTransfer = false;

      if (!shooterStingerAligned) {
        setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
      }

      // if stinger is at right position, set shooterStingerAligned to true
      if (Math.abs(stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
            && Math.abs(stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF) {
          shooterStingerAligned = true;
        }

      if (shooterStingerAligned && !correctShooterRpm) {
          shooter.setTargetRPM(10); //placeholder value
      }

      if (shooterStingerAligned && shooter.isReadyRPM()) {
            correctShooterRpm = true;
          }

      if (shooterStingerAligned && correctShooterRpm && !transferHasNote) {
        stinger.setRoller(-1);
        transfer.setSpeed(1);

        
        if (transfer.getTransferSensorValue() > 460) maxTransferSensorReached = true;
        
        if (maxTransferSensorReached && transfer.getTransferSensorValue() < 210) transferHasNote = true;
          
      }

      if (shooterStingerAligned && correctShooterRpm && transferHasNote) {
        transfer.setSpeed(0);
        shooter.setTargetRPM(0);
        stinger.setRoller(0);

        setBothPosPresets(StingerConfigs.STOW);

        shooterStingerAligned = false;
        correctShooterRpm = false;
        maxTransferSensorReached = false;
        transferHasNote = false;
        isDoneTransfer = true;
      }
    }
  }

  public void autoTransferFromShooter() {
    if (stinger.stingerRollerHasNote() && isDoneTransfer) return;

    else {
      isDoneTransfer = false;

      if (!shooterStingerAligned) {
        setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
      }
      
      // if stinger is at right position, set shooterStingerAligned to true
      if (Math.abs(stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
            && Math.abs(stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF) {
        shooterStingerAligned = true;
      }

      if (shooterStingerAligned && !correctShooterRpm) {
        shooter.setTargetRPM(10); //placeholder value
      }

      if (shooterStingerAligned && shooter.isReadyRPM()) {
        correctShooterRpm = true;
      }

      if (shooterStingerAligned && correctShooterRpm && !transferHasNote) {
        stinger.setRoller(1);
        transfer.setSpeed(-1);

        if (stinger.stingerRollerHasNote()) transferHasNote = true;
      }

      if (shooterStingerAligned && correctShooterRpm && transferHasNote) {
        transfer.setSpeed(0);
        shooter.setTargetRPM(0);
        stinger.setRoller(0);

        setBothPosPresets(StingerConfigs.STOW);

        shooterStingerAligned = false;
        correctShooterRpm = false;
        maxTransferSensorReached = false;
        transferHasNote = false;
        isDoneTransfer = true;
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // setBothPosPresets(StingerConfigs.STOW);
    // stinger.setRoller(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
      if (controls.getHumanStationIntake()) {
        setBothPosPresets(StingerConfigs.HUMAN_STATION_INTAKE);
      } else if (controls.getShooterStingerTransfer()) {
        setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
