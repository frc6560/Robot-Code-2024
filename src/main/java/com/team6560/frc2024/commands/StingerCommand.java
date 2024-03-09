// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

// import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Stinger;
import com.team6560.frc2024.subsystems.Transfer;
import com.team6560.frc2024.Constants.StingerConfigs;
// import com.team6560.frc2024.Constants.StingerConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class StingerCommand extends Command {
  /** Creates a new StingerCommand. */

  public static interface Controls {
    boolean getHumanStationIntake();

    boolean getShooterStingerTransfer();

    boolean getStingerShooterTransfer();

    boolean getAmpOuttake();

    boolean getManualStingerOuttake();
  }

  private final Stinger stinger;
  // private final Shooter shooter;
  private final Transfer transfer;
  private final Controls controls;

  // private boolean isDoneTransfer = true, shooterStingerAligned = false, correctShooterRpm = false,
  //     maxTransferSensorReached = false, transferHasNote = false;
  // private boolean stingerToIntakePos = false;

  public StingerCommand(Stinger stinger, Transfer transfer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stinger = stinger;
    // this.shooter = shooter;
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
    if (!stinger.stingerRollerHasNote()) {
      setBothPosPresets(StingerConfigs.STOW);
      stinger.setRoller(0.0);
    } else {
      setElevatorPosPresets(StingerConfigs.HUMAN_STATION_INTAKE);
      if (stinger.isStingerReady(StingerConfigs.HUMAN_STATION_INTAKE)) {
        stinger.setRoller(-1.0);
      } else {

      }
    }
  }

  public void autoTransferToShooter() {
    if (transfer.isInProximity()) {
      setBothPosPresets(StingerConfigs.STOW);
      stinger.setRoller(0.0);
    } else {
      setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
      if (!transfer.isInProximity()) {
        stinger.setRoller(-1.0);
      }
    }
  }

  public void autoTransferFromShooter() {
    if (stinger.stingerRollerHasNote()) {
      setBothPosPresets(StingerConfigs.STOW);
      stinger.setRoller(0.0);
    } else {
      setBothPosPresets(StingerConfigs.SHOOTER_TRANSFER);
      if (!stinger.stingerRollerHasNote()) {
        stinger.setRoller(1.0);
      }
    }
  }

  public void ampOuttake() {
    if (!stinger.stingerRollerHasNote()) {
      setBothPosPresets(StingerConfigs.STOW);
    } else {
      setBothPosPresets(StingerConfigs.AMP_OUTTAKE);
    }
    
    if (controls.getManualStingerOuttake()) {
      stinger.setRoller(-1.0); //TODO: choose whether to keep getManualStingerOuttake restricted to be only used in this function or move it into execute() for general use
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
      autoIntakeHumanStation();
    } else if (controls.getShooterStingerTransfer()) {
      autoTransferFromShooter();
    } else if (controls.getStingerShooterTransfer()) {
      autoTransferToShooter();
    } else if (controls.getAmpOuttake());
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
