// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public static interface Controls {
    boolean getAimShooter();

    boolean manualMode();
    double manualAim();
    double manualShooterSpeed();
  }
  private Shooter shooter;
  private Controls controls;

  private final double IDLE_RPM = 60;

  private boolean manualMode;

  public ShooterCommand(Shooter shooter, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.controls = controls;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTargetRPM(0);
    manualMode = true; //change later
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setManualAim(controls.manualAim());
    shooter.setTargetRPM(controls.manualShooterSpeed());
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
