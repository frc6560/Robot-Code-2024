// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj2.command.Command;

public class TrapCommand extends Command {
  /** Creates a new TrapCommand. */
  final Trap trap;
  final ManualControls controls;

  public TrapCommand(Trap trap, ManualControls controls) {
    this.trap = trap;
    this.controls = controls;

    addRequirements(trap);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // trap.setExtention(controls.getTrapExtention());
    // trap.setFeed(controls.getRunTrap());

    // trap.setAngle(controls.getTrapRotation());
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
