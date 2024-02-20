// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.subsystems.Transfer;

public class AutoTransferCommand extends Command {

  private final Transfer Transfer;
  /** Creates a new AutoTransferCommand. */
  public AutoTransferCommand(Transfer Transfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Transfer = Transfer;
    addRequirements(Transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Transfer.setIsAutoShooting(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transfer.setSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Transfer.setSpeed(0);
    Transfer.setIsAutoShooting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
