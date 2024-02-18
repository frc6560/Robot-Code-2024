// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team6560.frc2024.commands.ClimbCommand;

// import java.io.File;

import com.team6560.frc2024.commands.DriveCommand;
import com.team6560.frc2024.commands.IntakeCommand;
import com.team6560.frc2024.commands.ShooterCommand;
import com.team6560.frc2024.commands.TrapCommand;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Climb;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        final Limelight limelight;
        final Shooter shooter;
        final Intake intake;
        final Trap trap;
        final Climb climb;

        final Drivetrain drivetrain;
        private final DriveCommand driveCommand;
        private final ShooterCommand ShooterCommand;
        private final IntakeCommand intakeCommand;
        private final TrapCommand trapCommand;
        private final ClimbCommand climbCommand;
        

        private final ManualControls manualControls = new ManualControls(new XboxController(0), new XboxController(1));

        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                limelight = new Limelight();
                trap = new Trap();
                shooter = new Shooter(limelight, trap);
                intake = new Intake();
                drivetrain = new Drivetrain();
                climb = new Climb();


                driveCommand = new DriveCommand(drivetrain, limelight, manualControls);
                ShooterCommand = new ShooterCommand(shooter, trap, limelight, manualControls);
                intakeCommand = new IntakeCommand(intake, shooter, manualControls);
                trapCommand = new TrapCommand(trap, manualControls);
                climbCommand = new ClimbCommand(climb, manualControls);


                drivetrain.setDefaultCommand(driveCommand);
                shooter.setDefaultCommand(ShooterCommand);
                intake.setDefaultCommand(intakeCommand);
                trap.setDefaultCommand(trapCommand);
                climb.setDefaultCommand(climbCommand);

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
        }

        private void configureBindings() {
                SmartDashboard.putData("5 Balls", new PathPlannerAuto("5 Ball"));
                SmartDashboard.putData("Basic 2 Ball", new PathPlannerAuto("Basic 2 Ball"));
                SmartDashboard.putData("Short Line", new PathPlannerAuto("Short Lines"));
                SmartDashboard.putData("Copy of Short Line", new PathPlannerAuto("Copy of Short Lines"));
                SmartDashboard.putData("New Auto", new PathPlannerAuto("New Auto"));
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