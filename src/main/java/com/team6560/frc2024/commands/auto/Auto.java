// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands.auto;

import java.util.HashMap;
import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.SwerveAutoBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto extends Command {

  private HashMap<String, Command> eventMap;

  // private AutoBuilder autoBuilder;

  private Drivetrain drivetrain;

  /** Creates a new AutoBuilder. */
  public Auto(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    eventMap = new HashMap<>();

    this.drivetrain = drivetrain;

    AutoBuilder.configureHolonomic(
      () -> drivetrain.getOdometryPose2dNoApriltags(), // Pose2d supplier
      (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of
                                                // auto
      () -> drivetrain.getChassisSpeeds(),
      (robotRelativeSpeeds) -> drivetrain.driveRobotRelative(robotRelativeSpeeds),
      // Constants.m_kinematics, // SwerveDriveKinematics
      // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
      //                                   // and Y PID controllers)
      // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
      //                                   // rotation controller)
      // (state) -> drivetrain.setChassisState(state), // Module states consumer used to output to the drive
                                                    // subsystem
      Constants.pathFollowerConfig,
      // eventMap,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      drivetrain // The drive subsystem. Used to properly set the requirements of path following
                  // commands
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
