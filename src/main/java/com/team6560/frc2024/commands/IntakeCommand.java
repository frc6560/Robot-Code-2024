// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

  private final Intake Intake;

  // private final Transfer Transfer;

  private final Controls controls;

  public static interface Controls {
    boolean getIntakeIn();

    boolean getIntakeInReleased();

    boolean getIntakeOut();

    boolean getIntakeOutReleased();
  }
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake Intake, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake);
    this.Intake = Intake;
    this.controls = controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getIntakeIn()) {
      Intake.setSpeed(0.9);
    } else if (controls.getIntakeOut()) {
      Intake.setSpeed(-0.9);
    } else {
      Intake.setSpeed(0.0);
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
