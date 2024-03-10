// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.Constants.ShooterConfigs;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Transfer;

public class AutoInitCommand extends Command {
  /** Creates a new AutoShooterCommand. */

  private final Shooter shooter;
  private final Limelight limelight;
  private final Transfer transfer;
  private final Intake intake;

  private final int AUTO_SHOOTER_RPM = 2000;
  
  public AutoInitCommand(Shooter shooter, Limelight limelight, Transfer transfer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    
      this.shooter = shooter;
      this.limelight = limelight;
      this.transfer = transfer;
      this.intake = intake;
  
      addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTargetRPM(AUTO_SHOOTER_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.hasTarget()) {
        double[] shooterAim = ShooterConfigs.shooterMap.getRPMandAngle(limelight.getSpeakerDistance());;
        if (!shooterAim.equals(null)) {
          shooter.setTargetRPM(shooterAim[0]);
          if (transfer.isInProximity()) {
            shooter.setTargetAngle(shooterAim[1]);
          } else {
            shooter.setTargetAngle(Constants.MAX_ARC_ANGLE_FOR_INTAKE);
          }
        }
      } else {
        shooter.setTargetRPM(6000.0);
        shooter.setTargetAngle(Constants.MAX_ARC_ANGLE_FOR_INTAKE);
      }

      if (!transfer.isInProximity()) {
      intake.setSpeed(1);
      transfer.setSpeed(1.0);
    } else if (transfer.getIsAutoShooting()) {
      intake.setSpeed(0.0);
      transfer.setSpeed(1.0);
    } else {
      intake.setSpeed(0);
      transfer.setSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTargetRPM(0);
    intake.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
