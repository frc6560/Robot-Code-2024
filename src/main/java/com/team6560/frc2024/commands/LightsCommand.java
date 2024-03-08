// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Lights;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj2.command.Command;


public class LightsCommand extends Command {

  private final Lights light;
  private final Trap trap;
  private final Shooter shooter;
  private final Intake intake;
  private final ManualControls controls;

  /** Creates a new LightsCommand. */
  public LightsCommand(Lights light, Shooter shooter, Intake intake, Trap trap, ManualControls controls) {
    this.light = light;
    this.shooter = shooter;
    this.intake = intake;
    this.trap = trap;

    this.controls = controls;

    addRequirements(light);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    light.setLightsIdle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if()
    if (controls.getRunIntake()) {
      if(shooter.getTransferSensorTriggered()){
        light.setLightsShooting(shooter.readyToShoot(controls.getSafeAim()));
      } else {
        light.setLightsIntake();
      } 

    } else if (controls.getTrapPlace()){
      light.setLightsTrapPlace();
    
    } else if (controls.getTrapTransferIn()){
      light.setLightsTrapTransfer(trap.isAtTargetAngle() && trap.isAtTargetExtention() && !trap.getSensorTriggered());
    
    } else if (shooter.getTransferSensorTriggered() || trap.getSensorTriggered()){
      light.setLightsHasPiece();

    } else if (controls.getTrapIntake()){
      light.setLightsTrapIntake();

    } else if (controls.getReverseTransfer()){
      light.setLightsReverse();
    
    }
    else {
      light.setLightsIdle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    light.setLightsIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}