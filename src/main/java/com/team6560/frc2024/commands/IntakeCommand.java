// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
  final Intake intake;
  final Shooter shooter;
  final ManualControls controls;

  public IntakeCommand(Intake intake, Shooter shooter , ManualControls controls) {
    this.intake = intake;
    this.shooter = shooter;
    this.controls = controls;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeFeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.getRunIntake() && shooter.getShooterArcPosition() < Constants.SHOOTER_ARC_ACCEPTABLE_INTACE_POS){
      intake.setIntakeFeed(Constants.INTAKE_FEED_RATE);
      
    } else {
      intake.setIntakeFeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeFeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
