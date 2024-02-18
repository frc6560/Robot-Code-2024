// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import java.util.ResourceBundle.Control;

import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
  final Climb climb;
  final ManualControls controls;
  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climb climb, ManualControls controls) {
    this.climb = climb;
    this.controls = controls;

    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setClimbOutput(controls.getClimb());
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
