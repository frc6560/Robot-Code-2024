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
    double manualElevatorVelControl();
    double manualStingerAngleControl();

    boolean manualStow();
    boolean manualStingerIntakePos();
    boolean manualStingerShooterTransfer();
  }

  private final Stinger stinger;
  private final Shooter shooter;
  private final Transfer transfer;
  private final Controls controls;

  private boolean shooterStingerAligned = false, correctShooterRpm = false;



  public StingerCommand(Stinger stinger, Shooter shooter, Transfer transfer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stinger = stinger;
    this.shooter = shooter;
    this.transfer = transfer;
    this.controls = controls;

    addRequirements(stinger, shooter, transfer);
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

  public void autoTransferToShooter() { 
    if (!shooterStingerAligned) {
      setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
    }

    if (Math.abs(stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
          && Math.abs(stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF) {
        shooterStingerAligned = true;
      }

    if (shooterStingerAligned && !correctShooterRpm) {
        shooter.setTargetRPM(10); //placeholder value
    }

    if (shooterStingerAligned && shooter.isReadyManualAim()) {
          correctShooterRpm = true;
        }

    if (shooterStingerAligned && correctShooterRpm) {
      stinger.setRoller(-10);
      transfer.setSpeed(10);
    }

    if (shooterStingerAligned && correctShooterRpm && transfer.isInProximity()) {
      transfer.setSpeed(0);
      shooter.setTargetRPM(0);
      stinger.setRoller(0);

      shooterStingerAligned = false;
      correctShooterRpm = false;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setBothPosPresets(StingerConfigs.STOW);
    stinger.setRoller(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (transfer.isInProximity() || shooter.isReadyManualAim()) { //if there is a note in robot, stinger moves out of the way
      setBothPosPresets(StingerConfigs.STOW);
      stinger.setRoller(0);
      return;
    }

    if (controls.manualStingerIntakePos()) 
      setBothPosPresets(StingerConfigs.HUMAN_STATION_INTAKE);
    else if (controls.manualStingerShooterTransfer()) 
      setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
    else if (controls.manualStow()) 
      setBothPosPresets(StingerConfigs.STOW);



    // if (controls.manualStingerIntakePos()) 
    //   setBothPosPresets(StingerConfigs.HUMAN_STATION_INTAKE);
    // else if (controls.manualStingerShooterTransfer()) 
    //   setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
    // else setBothPosPresets(StingerConfigs.STOW);

    if (!controls.manualStow() && (controls.manualStingerIntakePos() || controls.manualStingerShooterTransfer()))
      stinger.setRoller(10.0 * (controls.manualStingerIntakePos() ? 1.0 : -1.0));
    else stinger.setRoller(0);
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
