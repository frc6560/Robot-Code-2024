// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.subsystems.Shooter;

public class AutoShooterCommand extends Command {
  /** Creates a new AutoShooterCommand. */

  private final Shooter shooter;

  private final int AUTO_SHOOTER_RPM = 2000;
  
  public AutoShooterCommand(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    
      this.shooter = shooter;
  
      addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTargetRPM(AUTO_SHOOTER_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTargetRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
