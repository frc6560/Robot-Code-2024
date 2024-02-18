// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.subsystems.Climb;
import com.team6560.frc2024.subsystems.Stinger;
import com.team6560.frc2024.subsystems.Shooter; 


public class AutoClimbCommand extends Command {
  private final Climb Climb;
  private final Stinger Stinger;
  private final Shooter Shooter;
  private final Controls controls;

  public static interface Controls{

  }
  
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(Climb Climb, Stinger Stinger, Shooter Shooter, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Climb);
    addRequirements(Stinger);
    addRequirements(Shooter);

    this.Climb = Climb;
    this.Stinger = Stinger;
    this.Shooter = Shooter;
    this.controls = controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
