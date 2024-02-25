// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands.Autos;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoShooter extends Command {
  final Shooter shooter;

  boolean isShooting = false;
  final double shootDuration = 30;
  double shootCounter = 0;
  
  /** Creates a new AutoShooterHead. */
  public AutoShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getTransferSensorTriggered() || isShooting){
      shooter.setArcPosition(Constants.SHOOTER_SUBWOOFER_POSITION);
      shooter.setRPM(Constants.SHOOTER_SUBWOOFER_RPM);
      
      System.out.println(shooter.readyToShoot());
      if(shooter.readyToShoot() || isShooting){
        shooter.setTransfer(Constants.TRANSFER_FEED_RATE);

        isShooting = true;
        shootCounter ++;
      }
    } else {
      shooter.setRPM(0.0);
      shooter.setTransfer(0.0);
      shooter.setArcPosition(Constants.SHOOTER_GROUND_INTAKE_POSITION);
    }

    if(shootDuration >= shootCounter){
      isShooting = false;
      shootCounter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0.0);
    shooter.setTransfer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
