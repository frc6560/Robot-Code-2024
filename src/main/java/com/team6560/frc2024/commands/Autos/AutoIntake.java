// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands.Autos;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  final Intake intake;
  final Shooter shooter;
  final Trap trap;
  
  
  public AutoIntake(Intake intake, Shooter shooter, Trap trap) {
    this.intake = intake;
    this.shooter = shooter;
    this.trap = trap;

    addRequirements(intake, shooter, trap);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trap.setExtention(-5);
    if(!shooter.getTransferSensorTriggered()){ 
      intake.setIntakeFeed(Constants.INTAKE_FEED_RATE);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
      shooter.setTransfer(Constants.TRANSFER_INTAKE_RATE);
    } else {
      intake.setIntakeFeed(0.0);
      shooter.setTransfer(0.0);
      shooter.setArcPosition(0);
      shooter.setRPM(0);//Constants.SHOOTER_SUBWOOFER_RPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTransfer(0.0);
    intake.setIntakeFeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getTransferSensorTriggered();
  }
}
