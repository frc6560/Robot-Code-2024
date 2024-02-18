// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.team6560.frc2024.commands.AutoIntakeCommand;
import com.team6560.frc2024.commands.AutoShooterCommand;
import com.team6560.frc2024.commands.AutoTransferCommand;

// import java.io.File;

import com.team6560.frc2024.commands.DriveCommand;
import com.team6560.frc2024.commands.IntakeCommand;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Climb;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Transfer;
import com.team6560.frc2024.subsystems.LightWorkNoReaction;
import com.team6560.frc2024.commands.LightWorkNoReactionCommand;
import com.team6560.frc2024.commands.ShooterCommand;


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
        private final Intake intake;
        private final Shooter shooter;
        private final Transfer transfer;
        // private final Climb climb; 
        private final Limelight limelight;
        private final LightWorkNoReaction lightWorkNoReaction;
        private final DriveCommand driveCommand;
        private final IntakeCommand intakeCommand;
        private final ShooterCommand shooterCommand;
       // private final AutoIntakeCommand autoIntakeCommand;
       // private final AutoShooterCommand autoShooterCommand;
      //  private final AutoTransferCommand autoTransferCommand;
        // private final LightWorkNoReactionCommand lightWorkNoReactionCommand;
      


        private final ManualControls manualControls = new ManualControls(new XboxController(0), new XboxController(1));

        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                limelight = new Limelight();
                drivetrain = new Drivetrain();
                shooter = new Shooter();
                intake = new Intake(shooter);
                transfer = new Transfer();
                // climb = new Climb();
                lightWorkNoReaction = new LightWorkNoReaction();
                driveCommand = new DriveCommand(drivetrain, manualControls, limelight);
                intakeCommand = new IntakeCommand(intake, transfer, manualControls);
                shooterCommand = new ShooterCommand(shooter, limelight, transfer, lightWorkNoReaction, manualControls);
              //  autoIntakeCommand = new AutoIntakeCommand(intake, transfer);
              //  autoShooterCommand = new AutoShooterCommand(shooter);
              //  autoTransferCommand = new AutoTransferCommand(transfer);
                // lightWorkNoReactionCommand = new LightWorkNoReactionCommand(lightWorkNoReaction, transfer, manualControls);

                drivetrain.setDefaultCommand(driveCommand);
                intake.setDefaultCommand(intakeCommand);
                shooter.setDefaultCommand(shooterCommand);
                // lightWorkNoReaction.setDefaultCommand(lightWorkNoReactionCommand);

                NamedCommands.registerCommand("print hello", Commands.print("hello"));
        //        NamedCommands.registerCommand("startShooter", autoShooterCommand);
        //        NamedCommands.registerCommand("startIntake", autoIntakeCommand);
        //        NamedCommands.registerCommand("shoot", autoTransferCommand.withTimeout(0.5));

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                System.out.println("\n\n\n\n\n\nAGHHHHHHHHHHH\n\n\n\n\n\n");
        }

        private void configureBindings() {
                SmartDashboard.putData("5 Balls", new PathPlannerAuto("5 Ball"));
                SmartDashboard.putData("Basic 2 Ball", new PathPlannerAuto("Basic 2 Ball"));
                // SmartDashboard.putData("Short Line", new PathPlannerAuto("Short Lines"));
                // SmartDashboard.putData("Copy of Short Line", new PathPlannerAuto("Copy of Short Lines"));
                SmartDashboard.putData("New Auto", new PathPlannerAuto("New Auto"));
                SmartDashboard.putData("New New Auto", new PathPlannerAuto("New New Auto"));
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