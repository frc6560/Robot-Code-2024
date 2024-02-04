// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

  private final Intake Intake;

  private final Transfer Transfer;

  private final Controls controls;

  public static interface Controls {
    boolean getIntake();
  }
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake Intake, Transfer Transfer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake);
    addRequirements(Transfer);
    this.Intake = Intake;
    this.Transfer = Transfer;
    this.controls = controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getIntake()) {
      Intake.setSpeed(0.5);
      if (!Transfer.isInProximity()) {
        Transfer.setSpeed(0.5);
      } else {
        Transfer.setSpeed(0);
      }
    } else {
      Intake.setSpeed(0);
      Transfer.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
