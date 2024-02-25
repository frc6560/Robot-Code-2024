// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.Constants.ClimbConfigs;
import com.team6560.frc2024.Constants.ClimbConstants;
import com.team6560.frc2024.Constants.StingerConfigs;
import com.team6560.frc2024.Constants.StingerConstants;
import com.team6560.frc2024.Constants.ShooterConfigs;
import com.team6560.frc2024.Constants.ShooterConstants;
import com.team6560.frc2024.subsystems.Climb;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Stinger;


import edu.wpi.first.wpilibj2.command.Command;



public class ClimbCommand extends Command {

  private final Climb climb;
  private final Shooter shooter;
  private final Stinger stinger;
  

  private final Controls controls; 

  public static interface Controls {
    double getClimbControls();
  }
  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climb climb, Controls controls, Shooter shooter, Stinger stinger) {
    this.climb = climb;
    this.shooter = shooter;
    this.stinger = stinger;
    this.controls = controls;  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  public void setClimbPreset(ClimbConfigs pos) {
    climb.setHeight(pos.getClimbPos());
  }

  public void autoClimbSequence() {
    boolean shooterDesiredPos = false, stingerInitDesiredPos = false, retractedClimb = false, shooterIsReady = false; 
    
    //Sets the angle of the shooter to prepare for climb.
    if (!shooterDesiredPos) {
      shooter.setTargetAngle(ShooterConfigs.CLIMB_ANGLE);
    }

    //if the shooter is in the right position set shooterDesiredPos to true.
    if (!shooterDesiredPos && shooter.getAngleDifference() < ShooterConstants.ACCEPTABLE_ANGLE_DIFF) {
      shooterDesiredPos = true;
    }

    if (shooterDesiredPos && !stingerInitDesiredPos) {
      stinger.setAngle(StingerConfigs.HUMAN_STATION_INTAKE.getStingerAngle());
    }
    
    // if shooterDesiredPos is true and stingerInitDesiredPos is false but in the right position set stingerInitDesiredPos to true.
    if(shooterDesiredPos && !stingerInitDesiredPos
    && (Math.abs(stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
    && Math.abs(stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF)) {
      stingerInitDesiredPos = true;
    }
    if(shooterDesiredPos && stingerInitDesiredPos && !retractedClimb) {
      climb.setHeight(ClimbConfigs.CLIMB_RETRACTED.getClimbPos());
    }
    
    // if shooterDesiredPos is true, stingerInitDesiredPos is true, and retractedClimb is false but in the right position set retractedClimb to true.
    if (shooterDesiredPos 
    && stingerInitDesiredPos
    && !retractedClimb
    && (Math.abs(climb.getVerticalPose() - ClimbConfigs.CLIMB_RETRACTED.getClimbPos()) < ClimbConstants.CLIMB_ACCEPTABLE_DIFF)) {
      retractedClimb = true;
    }
    
    if(shooterDesiredPos && stingerInitDesiredPos && retractedClimb) {
      shooter.setTargetAngle(StingerConfigs.SHOOT_IN_TRAP.getShooterAngle()); 
    }
    // if everything but shooterIsready is false, set shooterIsReady to true.
    if(shooterDesiredPos
     && stingerInitDesiredPos
     && retractedClimb 
     && !shooterIsReady 
     && ((Math.abs(stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
     && Math.abs(stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF))) {
      shooterIsReady = true;
    }

    //if everything is true, set the targetRPM to 10 
    if(shooterDesiredPos && stingerInitDesiredPos && retractedClimb && shooterIsReady) {
      shooter.setTargetRPM(10); //placeholder 
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // setClimbPreset(ClimbConfigs.CLIMB_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setHeightVelocity(controls.getClimbControls());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setHeightVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
