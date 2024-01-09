// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.utility;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** A RunCommand that keeps running, even when the robot is disabled.
 */
public class AlwaysRunInstantCommand extends InstantCommand {
  public AlwaysRunInstantCommand(Runnable runnable, Subsystem... requirements) {
    super(runnable, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
