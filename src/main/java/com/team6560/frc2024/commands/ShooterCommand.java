// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Shooter;
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

    // boolean getSetShootModeReleased();

    boolean setStowPos();

    // boolean manualMode();
    // double getManualAim();

    // double getManualShooterSpeed();

    boolean getIntakeIn();

    boolean getIntakeInReleased();

    boolean getIntakeOut();

    boolean getIntakeOutReleased();

    boolean getHumanStationIntake();

    boolean getShooterStingerTransfer();
  }

  private final Shooter Shooter;
  private final Limelight limelight;
  private final Controls controls;
  private final Transfer Transfer;
  private final LightWorkNoReaction Light;

  private boolean isShooting;

  // private final double IDLE_RPM = 60.0;
  // private boolean shooterAutoMoving;

  public ShooterCommand(Shooter Shooter, Limelight limelight, Transfer Transfer, LightWorkNoReaction Light,
      Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter = Shooter;
    this.limelight = limelight;
    this.Transfer = Transfer;
    this.controls = controls;
    this.Light = Light;

    this.isShooting = false;
  
    addRequirements(Shooter, Transfer, Light);
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
    shooterStuff();
    transferStuff();
  }

  public void shooterStuff() {
    if (controls.getSetShootMode()) {
      Light.setColorMode(CandleColorModes.SHOOT_MODE);
      isShooting = true;
      if (controls.getManualShootShooter()) {
        Transfer.setSpeed(1.0); // maybe add a down frames to fix not properly shooting the ring.
      } else {
        Transfer.setSpeed(0.0);
      }
      // if (Transfer.isInProximity() && Shooter.isReadyAutoAim()) {
      //   Transfer.setSpeed(1.0); // maybe add a down frames to fix not properly shooting the ring.
      // }
      if (limelight.hasTarget()) {
        double[] shooterAim = autoShooterAim();
        if (!shooterAim.equals(null)) {
          Shooter.setTargetRPM(shooterAim[0]);
          Shooter.setTargetAngle(shooterAim[1]);

          if (Transfer.isInProximity() && Shooter.isReadyRPMAndAngle()) {
            Transfer.setSpeed(1.0);
          }
        }
      } else {
        Shooter.setTargetRPM(6000.0);
        Shooter.setTargetAngle(Constants.MAX_ARC_ANGLE_FOR_INTAKE);
      }
    } else if (!controls.getSetShootMode() && isShooting) {
      isShooting = false;
      Light.setColorMode(CandleColorModes.NO_MODE);
      Shooter.setStowPos();
      Shooter.setTargetRPM(0.0);
      Transfer.setSpeed(0.0);
    } else if (controls.getHumanStationIntake()) {
      Shooter.setTargetAngle(StingerConfigs.HUMAN_STATION_INTAKE.getShooterAngle());
    } else if (controls.getShooterStingerTransfer()) {
      Shooter.setTargetAngle(StingerConfigs.SHOOTER_TRANSFER.getShooterAngle());
    }
  }

  public void transferStuff() {
    if (controls.getIntakeIn()) {
      if (!Transfer.isInProximity()) {
        Transfer.setSpeed(0.5);
      } else {
        Transfer.setSpeed(0);
      }
    } else if (controls.getIntakeOut()) {
      Transfer.setSpeed(-0.9);
    } else if (controls.getIntakeInReleased() || controls.getIntakeOutReleased()) {
      Transfer.setSpeed(0);
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
