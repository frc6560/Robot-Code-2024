// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.subsystems.Transfer;

public class AutoTransferCommand extends Command {

  private final Transfer transfer;

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Transfer");
  NetworkTableEntry isAutoShooting = ntTable.getEntry("isAutoShooting");

  /** Creates a new AutoTransferCommand. */
  public AutoTransferCommand(Transfer transfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.transfer = transfer;
    addRequirements(transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAutoShooting.setBoolean(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transfer.setSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setSpeed(0);
    isAutoShooting.setBoolean(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
