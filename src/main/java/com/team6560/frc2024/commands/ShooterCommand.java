// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2024.Constants.ShooterConfigs;
import com.team6560.frc2024.subsystems.Transfer;
import com.team6560.frc2024.Constants.CandleColorModes;
import com.team6560.frc2024.subsystems.LightWorkNoReaction;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public static interface Controls {

    boolean getManualShootShooter();

    boolean getSetShootMode();

    boolean getSetShootModeReleased();

    boolean setStowPos();

    // boolean manualMode();
    // double getManualAim();

    // double getManualShooterSpeed();
  }

  private final Shooter Shooter;
  private final Limelight limelight;
  private final Controls controls;
  private final Transfer Transfer;
  private final LightWorkNoReaction Light;

  private final double IDLE_RPM = 60.0;
  // private boolean shooterAutoMoving;

  public ShooterCommand(Shooter Shooter, Limelight limelight, Transfer Transfer, LightWorkNoReaction Light,
      Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter = Shooter;
    this.limelight = limelight;
    this.Transfer = Transfer;
    this.controls = controls;
    this.Light = Light;

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
    if (controls.getSetShootMode()) {
      Light.setColorMode(CandleColorModes.SHOOT_MODE);
      if (controls.getManualShootShooter() && Shooter.isReadyRPMAndAngle()) {
        Transfer.setSpeed(1.0); // maybe add a downframes to fix not properly shooting the ring.
      }
      // if (Transfer.isInProximity() && Shooter.isReadyAutoAim()) {
      //   Transfer.setSpeed(1.0); // maybe add a downframes to fix not properly shooting the ring.
      // }
      if (limelight.hasTarget()) {
        double[] shooterAim = autoShooterAim();
        if (!shooterAim.equals(null)) {
          Shooter.setTargetRPM(shooterAim[0]);
          Shooter.setTargetAngle(shooterAim[1]);
        }
      } else {
        Shooter.setTargetRPM(IDLE_RPM);
      }
    } else if (controls.getSetShootModeReleased()) {
      Light.setColorMode(CandleColorModes.NO_MODE);
      Shooter.setStowPos();
      Shooter.setTargetRPM(0.0);
      Shooter.setTargetAngle(0.0);
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
