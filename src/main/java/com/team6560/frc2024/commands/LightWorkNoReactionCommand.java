// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.Constants.CandleColorModes;
import com.team6560.frc2024.subsystems.LightWorkNoReaction;
import com.team6560.frc2024.subsystems.Transfer;




import edu.wpi.first.wpilibj2.command.Command;

public class LightWorkNoReactionCommand extends Command {
  /** Creates a new LightWorkNoSweatCommand. */

  private final LightWorkNoReaction light;
  private final Transfer transfer;
  private final Controls controls;

  public static interface Controls {
    boolean getIntakeIn();

    boolean getIntakeInReleased();

    boolean getIntakeOut();

    boolean getIntakeOutReleased();
  }

  public LightWorkNoReactionCommand(LightWorkNoReaction light, Transfer transfer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(light);
    this.light = light;
    this.transfer = transfer;
    this.controls = controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getIntakeInReleased() || controls.getIntakeOutReleased()) {
      light.setColorMode(CandleColorModes.NO_MODE);
    }
    if (controls.getIntakeIn() || controls.getIntakeOut()) {
      light.setColorMode(CandleColorModes.INTAKE_MODE);
    }
    else if (transfer.isInProximity()) {
      light.setColorMode(CandleColorModes.HOLD_MODE);
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
