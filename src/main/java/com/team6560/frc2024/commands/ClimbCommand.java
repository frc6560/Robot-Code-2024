// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands;

import com.team6560.frc2024.Constants.ClimbConfigs;
import com.team6560.frc2024.Constants.StingerConfigs;
import com.team6560.frc2024.subsystems.Climb;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Stinger;
import com.team6560.frc2024.subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.Command;



public class ClimbCommand extends Command {

  private final Climb Climb;
  private final Shooter Shooter;
  private final Stinger Stinger;
  private final Transfer Transfer;

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
  public ClimbCommand(Climb Climb, Controls controls, Shooter Shooter, Stinger Stinger, Transfer Transfer) {
    this.Climb = Climb;
    this.Shooter = Shooter;
    this.Stinger = Stinger;
    this.Transfer = Transfer;
    this.controls = controls;  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Climb, Shooter, Stinger, Transfer);
  }

  public void setClimbPreset(ClimbConfigs pos) {
    Climb.setHeight(pos.getClimbPos());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setClimbPreset(ClimbConfigs.CLIMB_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Climb.setHeightVelocity(controls.getClimbControls());

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
