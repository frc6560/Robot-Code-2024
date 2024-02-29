// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj2.command.Command;

public class TrapCommand extends Command {
  final Trap trap;
  final Shooter shooter;
  final ManualControls controls;



  public TrapCommand(Trap trap, Shooter shooter, ManualControls controls) {
    this.trap = trap;
    this.controls = controls;
    this.shooter = shooter;

    addRequirements(trap);
  }

  @Override
  public void initialize() {
  }
  
  boolean taskCompleted = false;

  @Override
  public void execute() {

    if(controls.getTrapIntake() && !taskCompleted){
      taskCompleted = intakeFromWall();

    } else if(controls.getTrapPlace() && !taskCompleted){
      taskCompleted = placeNote(controls.getShoot());

    } else if(controls.getTrapTransferIn() && !taskCompleted){
      taskCompleted = transferFromShooter();

    } else{
      goToStow();

      if(!(controls.getTrapIntake() || controls.getTrapPlace() || controls.getTrapTransferIn() || controls.getTrapTransferOut()))
        taskCompleted = false;
    }
  }

  public boolean goToStow(){
    trap.setFeed(0.0);
    trap.setAngle(Constants.TRAP_STOW_ANGLE);

    if(trap.isAtTargetAngle()){
      trap.setExtention(Constants.TRAP_STOW_EXTENTION);
    }

    return trap.isAtTargetAngle() && trap.isAtTargetExtention();
  }

  public boolean intakeFromWall(){
    trap.setAngle(Constants.TRAP_WALL_ANGLE);
    trap.setExtention(Constants.TRAP_WALL_EXTENTION);
    
    if(!trap.getSensorTriggered()) {
      trap.setFeed(Constants.TRAP_WALL_FEED_RATE);
    } else {
      trap.setFeed(0.0);
    }

    return trap.getSensorTriggered();
  }

  public boolean transferFromShooter(){
    trap.setAngle(Constants.TRAP_HANDOFF_ANGLE);
    trap.setExtention(Constants.TRAP_HANDOFF_EXTENTION);
    
    if(!trap.getSensorTriggered()) {
      trap.setFeed(Constants.TRAP_TRANSFER_IN_FEED_RATE);
    } else {
      trap.setFeed(0.0);
    }

    return trap.getSensorTriggered();
  }

  public boolean transferToShooter(boolean runFeed){
    trap.setAngle(Constants.TRAP_HANDOFF_ANGLE);
    trap.setExtention(Constants.TRAP_HANDOFF_EXTENTION);

    if(runFeed){
      trap.setFeed(Constants.TRAP_TRANSFER_OUT_FEED_RATE);
    } else {
      trap.setFeed(0.0);
    }

    return trap.isAtTargetAngle() && trap.isAtTargetExtention() && !trap.getSensorTriggered();
  }

  public boolean placeNote(boolean runFeed){
    trap.setAngle(Constants.TRAP_AMP_ANGLE);
    trap.setExtention(Constants.TRAP_AMP_EXTENTION);

    if(runFeed && trap.isAtTargetAngle() && trap.isAtTargetExtention()){// && shooter.isAtTargetArcAngle()){
      trap.setFeed(Constants.TRAP_AMP_FEED_RATE);
    } else {
      trap.setFeed(0);
    }

    return false;// !trap.getSensorTriggered();
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
