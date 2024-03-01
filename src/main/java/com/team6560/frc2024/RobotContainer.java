// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team6560.frc2024.commands.ClimbCommand;

// import java.io.File;

import com.team6560.frc2024.commands.DriveCommand;
import com.team6560.frc2024.commands.IntakeCommand;
import com.team6560.frc2024.commands.LightsCommand;
import com.team6560.frc2024.commands.ShooterCommand;
import com.team6560.frc2024.commands.TrapCommand;
import com.team6560.frc2024.commands.Autos.AutoIntake;
import com.team6560.frc2024.commands.Autos.AutoShooter;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Climb;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Lights;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;
import com.team6560.frc2024.subsystems.Trap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        final Limelight limelight;
        final Shooter shooter;
        final Intake intake;
        final Trap trap;
        final Climb climb;
        final Lights lights;

        final Drivetrain drivetrain;
        private final DriveCommand driveCommand;
        private final ShooterCommand ShooterCommand;
        private final IntakeCommand intakeCommand;
        private final TrapCommand trapCommand;
        private final ClimbCommand climbCommand;
        private final LightsCommand lightsCommand;
        

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
                lights = new Lights();

                NamedCommands.registerCommand("Intake", new AutoIntake(intake, shooter));
                // NamedCommands.registerCommand("Aim", new AutoShooter(shooter, limelight, drivetrain, false));
                NamedCommands.registerCommand("Shoot", new AutoShooter(shooter, limelight));


                driveCommand = new DriveCommand(drivetrain, limelight, manualControls);
                ShooterCommand = new ShooterCommand(shooter, trap, limelight, manualControls);
                intakeCommand = new IntakeCommand(intake, shooter, manualControls);
                trapCommand = new TrapCommand(trap, shooter, manualControls);
                climbCommand = new ClimbCommand(climb, manualControls);
                lightsCommand = new LightsCommand(lights, shooter, intake, trap, manualControls);


                drivetrain.setDefaultCommand(driveCommand);
                shooter.setDefaultCommand(ShooterCommand);
                intake.setDefaultCommand(intakeCommand);
                trap.setDefaultCommand(trapCommand);
                climb.setDefaultCommand(climbCommand);
                lights.setDefaultCommand(lightsCommand);

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
                
                autoChooser.addOption("One Still", oneBall());
                autoChooser.addOption("One Taxi", oneBallTaxi());
                autoChooser.addOption("Two", twoBall());
                autoChooser.addOption("Three", threeBall());
                autoChooser.addOption("five", farThreeBall());
                autoChooser.addOption("red five", farThreeBallRed());
                autoChooser.addOption("Two mid", twoBallMid());


                // Command combo4 = (new PathPlannerAuto("Zero")).andThen(new AutoShooter(shooter, limelight, 19.75, 5800))
                // .andThen(new PathPlannerAuto("One")).andThen(new AutoShooter(shooter, limelight, 15  , 5500))
                // .andThen(new PathPlannerAuto("Two")).andThen(new AutoShooter(shooter, limelight, 18.85, 5500))
                // .andThen(new PathPlannerAuto("Three")).andThen(new PathPlannerAuto("Four")).andThen(new AutoShooter(shooter, limelight, 28, 5800));

                autoChooser.setDefaultOption("Three", threeBall());
        }


        public Command farThreeBall() {
                var alliance = DriverStation.getAlliance();
                String prefix = "";
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
                        prefix = "R";                }


                Command combo = (new AutoShooter(shooter, limelight, 45, 5000))
                        .andThen(new PathPlannerAuto("F1")).withTimeout(5)
                        .andThen(new PathPlannerAuto("straight")).andThen(new AutoShooter(shooter, limelight, 19, 5200));
                        // .andThen(new PathPlannerAuto("F2")).withTimeout(7)
                        // .andThen(new PathPlannerAuto("F3")).andThen(new AutoShooter(shooter,limelight,20,5200));

                return combo;
        }


        public Command farThreeBallRed() {
                Command combo = (new AutoShooter(shooter, limelight, 45, 5000))
                        .andThen(new PathPlannerAuto("RF1")).withTimeout(5)
                        .andThen(new PathPlannerAuto("straight")).andThen(new AutoShooter(shooter, limelight, 19, 5200));
                        // .andThen(new PathPlannerAuto("RF2")).withTimeout(7)
                        // .andThen(new PathPlannerAuto("RF3")).andThen(new AutoShooter(shooter,limelight,20,5200));

                return combo;
        }
        
        public Command oneBall(){
                Command combo = new AutoShooter(shooter, limelight, 45, 5000);
                return combo;
        }
             
        public Command oneBallTaxi(){
                Command combo = (new PathPlannerAuto("Zero")).andThen(new AutoShooter(shooter, limelight, 45, 5000));
                return combo;
        }
        public Command twoBall(){
                Command combo = oneBallTaxi()
                .andThen(new PathPlannerAuto("One")).andThen(new AutoShooter(shooter, limelight, 22 , 5200));
                return combo;
        }

        public Command threeBall(){
                Command combo = twoBall()
                        .andThen(new PathPlannerAuto("Two")).andThen(new AutoShooter(shooter, limelight, 24, 5200));
                return combo;
        }

        public Command twoBallMid(){
                Command combo = (new PathPlannerAuto("TwoMid")).andThen(new AutoShooter(shooter, limelight, 22, 5200));
                return combo;
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