// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands.auto;

// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.SwerveAutoBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPoint;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto {

  // private HashMap<String, Command> eventMap;

  // private AutoBuilder autoBuilder;

  // private Drivetrain drivetrain;

  /** Creates a new AutoBuilder. */
  public Auto(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    // eventMap = new HashMap<>();

    // this.drivetrain = drivetrain;

    AutoBuilder.configureHolonomic(
        () -> drivetrain.getOdometryPose2dNoApriltags(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of
                                                  // auto
        () -> drivetrain.getChassisSpeeds(),
        (robotRelativeSpeeds) -> drivetrain.driveRobotRelative(robotRelativeSpeeds),
        // Constants.m_kinematics, // SwerveDriveKinematics
        // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation
        // error (used to create the X
        // // and Y PID controllers)
        // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation
        // error (used to create the
        // // rotation controller)
        // (state) -> drivetrain.setChassisState(state), // Module states consumer used
        // to output to the drive
        // subsystem
        Constants.pathFollowerConfig,
        // eventMap,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
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

  public Command get5Balls() {
    PathPlannerPath a5Ball = PathPlannerPath.fromPathFile("5 Ball");
    return AutoBuilder.followPath(a5Ball);
  }

  public Command goToPose(Pose2d desiredPose) {

    // Pose2d currPose = drivetrain.getPose();

    // ChassisSpeeds currChassisSpeeds = drivetrain.getChassisSpeeds();

    // double currSpeed = Math
    // .abs(Math.hypot(currChassisSpeeds.vxMetersPerSecond,
    // currChassisSpeeds.vyMetersPerSecond));

    // Rotation2d heading = Rotation2d
    // .fromRadians(Math.atan2(currChassisSpeeds.vyMetersPerSecond,
    // currChassisSpeeds.vxMetersPerSecond));

    return AutoBuilder.pathfindToPose(desiredPose, new PathConstraints(1.0, 1.0, 1.0, 1.0));
  }
}
