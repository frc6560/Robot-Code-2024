// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2024.subsystems.Transfer;



public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public static interface Controls {
    boolean getAimShooter();

    // boolean manualMode();
    double getManualAim();
    double getManualShooterSpeed();
  }
  private Shooter Shooter;
  private Controls controls;
  private Transfer Transfer;

  // private final double IDLE_RPM = 60;

  // private boolean manualMode;

  public ShooterCommand(Shooter Shooter, Transfer Transfer, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter = Shooter;
    this.Transfer = Transfer;
    this.controls = controls;

    addRequirements(Shooter);
    addRequirements(Transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setTargetRPM(0);
    // manualMode = true; //change later
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getAimShooter() && Transfer.isInProximity() && Shooter.isReady()) {
      Transfer.setSpeed(1.0); //maybe add a downframes to fix not properly shooting the ring.
    }
    Shooter.setManualAim(controls.getManualAim());
    Shooter.setTargetRPM(controls.getManualShooterSpeed());
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
