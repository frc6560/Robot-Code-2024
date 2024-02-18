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

  private final Climb Climb;
  private final Shooter Shooter;
  private final Stinger Stinger;
  

  private final Controls controls; 

  public static interface Controls {
    double getClimbControls(); 
    double manualElevatorVelControl();
    double manualStingerAngleControl();
  
    boolean manualStow();
    boolean manualStingerIntakePos();
    boolean manualStingerShooterTransfer();
  }
  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climb Climb, Controls controls, Shooter Shooter, Stinger Stinger) {
    this.Climb = Climb;
    this.Shooter = Shooter;
    this.Stinger = Stinger;
    this.controls = controls;  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Climb, Shooter, Stinger);
  }

  public void setClimbPreset(ClimbConfigs pos) {
    Climb.setHeight(pos.getClimbPos());
  }

  public void autoClimbSequence() {
    boolean shooterDesiredPos = false, stingerInitDesiredPos = false, retractedClimb = false, shooterIsReady = false; 
    
    if (!shooterDesiredPos) {
      Shooter.setTargetAngle(ShooterConfigs.CLIMB_ANGLE);
    }

    if (!shooterDesiredPos && Shooter.getAngleDifference() < ShooterConstants.ACCEPTABLE_ANGLE_DIFF) {
      shooterDesiredPos = true;
    }

    if (shooterDesiredPos && !stingerInitDesiredPos) {
      Stinger.setAngle(StingerConfigs.HUMAN_STATION_INTAKE.getStingerAngle());
    }

    if(shooterDesiredPos && !stingerInitDesiredPos
    && (Math.abs(Stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
    && Math.abs(Stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF)) {
      stingerInitDesiredPos = true;
    }
    if(shooterDesiredPos && stingerInitDesiredPos && !retractedClimb) {
      Climb.setHeight(ClimbConfigs.CLIMB_RETRACTED.getClimbPos());
    }

    if (shooterDesiredPos 
    && stingerInitDesiredPos
    && !retractedClimb
    && (Math.abs(Climb.getVerticalPose() - ClimbConfigs.CLIMB_RETRACTED.getClimbPos()) < ClimbConstants.CLIMB_ACCEPTABLE_DIFF)) {
      retractedClimb = true;
    }
    
    if(shooterDesiredPos && stingerInitDesiredPos && retractedClimb) {
      Shooter.setTargetAngle(StingerConfigs.SHOOT_IN_TRAP.getStingerAngle()); 
    }
  
    if(shooterDesiredPos
     && stingerInitDesiredPos
     && retractedClimb 
     && !shooterIsReady 
     && ((Math.abs(Stinger.getAngle() - StingerConfigs.SHOOTER_TRANSFER.getStingerAngle()) < StingerConstants.STINGER_ANGLE_ACCEPTABLE_DIFF
     && Math.abs(Stinger.getExtension() - StingerConfigs.SHOOTER_TRANSFER.getElevatorPos()) < StingerConstants.STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF))) {
      shooterIsReady = true;
    }

    if(shooterDesiredPos && stingerInitDesiredPos && retractedClimb && shooterIsReady) {
      Shooter.setTargetRPM(10); //placeholder 
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setClimbPreset(ClimbConfigs.CLIMB_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Climb.setHeightVelocity(controls.getClimbControls());
    Climb.setSpeed(controls.getClimbControls());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climb.setHeightVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
