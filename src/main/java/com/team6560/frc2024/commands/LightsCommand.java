// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Lights;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj2.command.Command;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;


public class LightsCommand extends Command {

  private final Lights light;
  private final Limelight limelight;
  private final Trap trap;
  private final Shooter shooter;
  private final Intake intake;
  private final ManualControls controls;

  int strobeTimer = 30;
  int strobeDuration = 20;

  boolean rumbling = false;
  boolean firstFrame = false;

  /** Creates a new LightsCommand. */
  public LightsCommand(Lights light, Limelight limelight, Shooter shooter, Intake intake, Trap trap, ManualControls controls) {
    this.light = light;
    this.limelight = limelight;
    this.shooter = shooter;
    this.intake = intake;
    this.trap = trap;

    this.controls = controls;

    addRequirements(light);

    ntDispTab("Lights")
      .add("Strobing", this::strobing);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    light.setLightsIdle();
  }

  private boolean strobing(){
    return strobeTimer < strobeDuration;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getProximitySensor()){
      strobeTimer = 0;
    } else {
      strobeTimer += 1;
    }

    // if(shooter.readyToShoot(controls.getSafeAim())){
    //   if(!firstFrame){
    //     firstFrame = true;
    //     rumbling = true;
    //   } else {
    //     rumbling = false;
    //   }
    // } else {
    //   firstFrame = false;
    //   rumbling = false;
    // }

    if(strobing() || rumbling){
      if(strobing()){
        limelight.setLightMode(2);
      }

      controls.setXboxRumble(0.25);
      controls.setControlStationRumble(0.25);

    } else {
      limelight.setLightMode(1);
      controls.setXboxRumble(0);
      controls.setControlStationRumble(0);
    }
    

    if (controls.getRunIntake()) {
      if(shooter.getTransferSensorTriggered() || intake.getProximitySensor()){
        light.setLightsShooting(shooter.readyToShoot(controls.getSafeAim()));
      } else {
        light.setLightsIntake();
      } 

    } else if (controls.getAmpPlace()){
      light.setLightsTrapPlace();
    
    // } else if (controls.getTrapTransferIn()){
    //   light.setLightsTrapTransfer(trap.isAtTargetAngle() && trap.isAtTargetExtention() && !trap.getSensorTriggered());
    
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