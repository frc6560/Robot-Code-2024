// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Stinger;
import com.team6560.frc2024.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.Constants.ShooterConfigs;
import com.team6560.frc2024.Constants.StingerConfigs;
import com.team6560.frc2024.subsystems.Transfer;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.Constants.CandleColorModes;
import com.team6560.frc2024.subsystems.LightWorkNoReaction;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public static interface Controls {

    boolean getManualShootShooter();

    boolean getSetShootMode();

    boolean getSetShootModeReleased();

    boolean setStowPos();

    boolean getIntakeIn();

    boolean getIntakeInReleased();

    boolean getIntakeOut();

    boolean getIntakeOutReleased();

    // boolean getHumanStationIntake();

    boolean getShooterStingerTransfer();

    boolean getStingerShooterTransfer();
  }

  private final Shooter shooter;
  private final Limelight limelight;
  private final Stinger stinger;
  private final Controls controls;
  private final Transfer transfer;
  private final LightWorkNoReaction light;

  // private boolean isShooting;

  public ShooterCommand(Shooter shooter, Limelight limelight, Stinger stinger, Transfer transfer,
      LightWorkNoReaction light, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.limelight = limelight;
    this.stinger = stinger;
    this.transfer = transfer;
    this.controls = controls;
    this.light = light;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public double[] autoShooterAim() {

    return ShooterConfigs.shooterMap.getRPMandAngle(limelight.getSpeakerDistance());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This method will be called once per scheduler run
    shooterStuff();
    transferStuff();
  }

  public void shooterStuff() {
    if (controls.getIntakeIn() || controls.getIntakeOut()) {
      shooter.setTargetAngle(0);
    }
    else if (controls.getSetShootMode()) {
      light.setColorMode(CandleColorModes.SHOOT_MODE);
      // isShooting = true;
      if (controls.getManualShootShooter()) {
        transfer.setSpeed(1.0); // maybe add a down frames to fix not properly shooting the ring.
      } else {
        transfer.setSpeed(0.0);
      }
      // if (Transfer.isInProximity() && Shooter.isReadyAutoAim()) {
      // Transfer.setSpeed(1.0); // maybe add a down frames to fix not properly
      // shooting the ring.
      // }
      if (limelight.hasTarget()) {
        double[] shooterAim = autoShooterAim();
        if (!shooterAim.equals(null)) {
          shooter.setTargetRPM(shooterAim[0]);
          shooter.setTargetAngle(shooterAim[1]);

          if (transfer.isInProximity() && shooter.isReadyRPMAndAngle()) {
            transfer.setSpeed(1.0);
          }
        }
      } else {
        shooter.setTargetRPM(6000.0);
        shooter.setTargetAngle(Constants.MAX_ARC_ANGLE_FOR_INTAKE);
      }
    } else if (controls.getSetShootModeReleased()) {
      light.setColorMode(CandleColorModes.NO_MODE);
      shooter.setStowPos();
      shooter.setTargetRPM(0.0);
      transfer.setSpeed(0.0);


    } else if (controls.getShooterStingerTransfer()) {
      shooter.setTargetAngle(StingerConfigs.SHOOTER_TRANSFER.getShooterAngle());
      if (stinger.isStingerReady(StingerConfigs.SHOOTER_TRANSFER)) {
        shooter.setTargetRPM(10.0); // placeholder value
        if (shooter.isReadyRPM()) {
          while (!stinger.stingerRollerHasNote()) {
            transfer.setSpeed(1.0);
          }
          transfer.setSpeed(0.0);
          shooter.setTargetRPM(0.0);
        }
      }

      
    } else if (controls.getStingerShooterTransfer()) {
      shooter.setTargetAngle(StingerConfigs.SHOOTER_TRANSFER.getShooterAngle());
      if (stinger.isStingerReady(StingerConfigs.SHOOTER_TRANSFER)) {
        shooter.setTargetRPM(-10.0); // placeholder value
        if (shooter.isReadyRPM()) {
          while (transfer.getTransferSensorValue() < 460) {
            transfer.setSpeed(-1.0);
          }
          if (transfer.getTransferSensorValue() < 300) { // this is to prevent a situation where the color sensor detects the "bottom" of the note and stopping the transfer motor while the note still is in contact with shooter wheels
            transfer.setSpeed(0);
            shooter.setTargetRPM(0);
          }
        }
      }
    }
  }
  

  public void transferStuff() {
    if (controls.getIntakeIn()) {
      if (!transfer.isInProximity()) {
        transfer.setSpeed(0.5);
      } else {
        transfer.setSpeed(0);
      }
    } else if (controls.getIntakeOut()) {
      transfer.setSpeed(-0.9);
    } else if (controls.getIntakeInReleased() || controls.getIntakeOutReleased()) {
      transfer.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
