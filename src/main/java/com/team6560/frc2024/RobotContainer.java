// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

// import java.io.File;

import com.team6560.frc2024.commands.DriveCommand;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        final Drivetrain drivetrain;
        private final DriveCommand driveCommand;

        private final ManualControls manualControls = new ManualControls(new XboxController(0), new XboxController(1));

        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain = new Drivetrain();
                driveCommand = new DriveCommand(drivetrain, manualControls);

                drivetrain.setDefaultCommand(driveCommand);

                NamedCommands.registerCommand("print hello", Commands.print("hello"));

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
        }

        private void configureBindings() {
                SmartDashboard.putData("5 Balls", new PathPlannerAuto("5 Ball"));
                SmartDashboard.putData("Basic 2 Ball", new PathPlannerAuto("Basic 2 Ball"));
                SmartDashboard.putData("Short Line", new PathPlannerAuto("Short Lines"));
                SmartDashboard.putData("Copy of Short Line", new PathPlannerAuto("Copy of Short Lines"));
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

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}