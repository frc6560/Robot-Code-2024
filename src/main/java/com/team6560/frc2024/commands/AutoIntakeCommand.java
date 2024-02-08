// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Transfer;

public class AutoIntakeCommand extends Command {

  private final Intake Intake;
  private final Transfer Transfer;
  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(Intake Intake, Transfer Transfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = Intake;
    this.Transfer = Transfer;
    addRequirements(Intake);
    addRequirements(Transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Transfer.isInProximity()) {
      Intake.setSpeed(1);
    } else if (!Transfer.getIsAutoShooting()) {
      Intake.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
