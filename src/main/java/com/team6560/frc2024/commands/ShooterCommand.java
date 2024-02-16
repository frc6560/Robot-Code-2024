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

    boolean aButtonSetManualMode();

    boolean setStowPos();

    // boolean manualMode();
    double getManualAim();

    double getManualShooterSpeed();
  }

  private final Shooter Shooter;
  // private final Limelight limelight;
  private final Controls controls;
  private final Transfer Transfer;
  private final LightWorkNoReaction Light;

  // private final double IDLE_RPM = 60;

  private boolean manualMode;

  public ShooterCommand(Shooter Shooter, Transfer Transfer, LightWorkNoReaction Light,  Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter = Shooter;
    // this.limelight = limelight;
    this.Transfer = Transfer;
    this.controls = controls;
    this.Light = Light;
    this.manualMode = true;

    addRequirements(Shooter, Transfer, Light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setTargetRPM(0);
    // manualMode = true; //change later
  }

  public void autoShooterAim() {
     double[] shooterData = ShooterConfigs.shooterMap.get(limelight.getDistance());
     Shooter.setTargetRPM(shooterData[1]);
     Shooter.setTargetAngle(shooterData[2]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.aButtonSetManualMode() && !manualMode) {
      manualMode = !manualMode;
      Light.setColorMode(CandleColorModes.NO_MODE);
    } else if (controls.aButtonSetManualMode()) {
      manualMode = !manualMode;
    }
    if (manualMode) {
      if (controls.setStowPos()) {
        Shooter.setStowPos();
      } else {
        Shooter.setManualAim(controls.getManualAim());
        Shooter.setTargetRPM(controls.getManualShooterSpeed());
        if (controls.getManualShootShooter() && Transfer.isInProximity() && Shooter.isReadyManualAim()) {
        Transfer.setSpeed(1.0); // maybe add a downframes to fix not properly shooting the ring.
        }
      }
    } else {
      Light.setColorMode(CandleColorModes.SHOOT_MODE);
      if (Transfer.isInProximity() && Shooter.isReadyAutoAim()) {
        Transfer.setSpeed(1.0); // maybe add a downframes to fix not properly shooting the ring.
      }
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
