// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

                NamedCommands.registerCommand("Intake", new AutoIntake(intake, shooter, trap));
                // NamedCommands.registerCommand("Aim", new AutoShooter(shooter, limelight, drivetrain, false));
                // NamedCommands.registerCommand("Shoot", new AutoShooter(shooter, limelight));


                driveCommand = new DriveCommand(drivetrain, shooter, limelight, manualControls);
                ShooterCommand = new ShooterCommand(shooter, trap, limelight, manualControls);
                intakeCommand = new IntakeCommand(intake, shooter, manualControls);
                trapCommand = new TrapCommand(trap, shooter, manualControls);
                climbCommand = new ClimbCommand(climb, manualControls);
                lightsCommand = new LightsCommand(lights, limelight, shooter, intake, trap, manualControls);


                drivetrain.setDefaultCommand(driveCommand);
                shooter.setDefaultCommand(ShooterCommand);
                intake.setDefaultCommand(intakeCommand);
                trap.setDefaultCommand(trapCommand);
                climb.setDefaultCommand(climbCommand);
                lights.setDefaultCommand(lightsCommand);


                autoChooser = new SendableChooser<Command>();
                // autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
                SmartDashboard.putNumber("Auto Delay", 0);
                


                autoChooser.addOption("Four ball", fourBall());

                autoChooser.addOption("Robin Hood", robinHood());

                autoChooser.addOption("Green Arrow", greenArrow());

                autoChooser.addOption("One Still", OneStill());

                autoChooser.addOption("One Taxi", OneTaxi());

                autoChooser.addOption("Two Mid", TwoMid());

                autoChooser.addOption("Calibration", Calibration());
        }

        public Command Calibration(){
                return (new PathPlannerAuto("cali1")).andThen(new WaitCommand(3)).andThen(new PathPlannerAuto("cali2"));
        }

        public Command fourBall(){
                return (new AutoShooter(shooter, limelight, 44, 4000))
                                .andThen(new PathPlannerAuto("Four1")
                                .andThen((new PathPlannerAuto("Four2").withTimeout(2))
                                .andThen(new AutoShooter(shooter, limelight, drivetrain)))
                        .andThen(new PathPlannerAuto("Four3"))
                        .andThen((new PathPlannerAuto("Four3.5")).withTimeout(2))
                                .andThen(new AutoShooter(shooter, limelight, drivetrain))
                                
                        .andThen(new PathPlannerAuto("Four4"))
                        .andThen(new PathPlannerAuto("Four5"))
                                .andThen(new AutoShooter(shooter, limelight, drivetrain)));
        }

        public Command robinHood(){
                return (new AutoShooter(shooter, limelight, 44, 4000))
                                .andThen(new PathPlannerAuto("RH1")).andThen(new WaitCommand(0.2))
                                .andThen(new PathPlannerAuto("RH2")).andThen(new WaitCommand(0.2))

                        .andThen(new AutoShooter(shooter, limelight, drivetrain)).andThen(new WaitCommand(0.2))
                                .andThen(new PathPlannerAuto("RH3")).andThen(new WaitCommand(0.1))
                                .andThen(new PathPlannerAuto("RH4"))

                        .andThen(new AutoShooter(shooter, limelight, drivetrain))
                        ;
        }

        public Command greenArrow(){
                return (new AutoShooter(shooter, limelight, 44, 4000))
                                .andThen(new PathPlannerAuto("GA1")).andThen(new WaitCommand(0.1))
                                .andThen(new PathPlannerAuto("GA2")).andThen(new WaitCommand(0.5))

                        .andThen(new AutoShooter(shooter, limelight, drivetrain))
                                .andThen(new PathPlannerAuto("GA3")).andThen(new WaitCommand(0.1))
                                .andThen(new PathPlannerAuto("GA4")).andThen(new WaitCommand(0.3))

                        .andThen(new AutoShooter(shooter, limelight, drivetrain));
        }

        public Command OneStill(){
                return (new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0))).andThen(new AutoShooter(shooter, limelight, 44, 4000));
        }

        public Command OneTaxi(){
                return (new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0))).andThen
                (new AutoShooter(shooter, limelight, 44, 4000)).withTimeout(7)
                                .andThen(new PathPlannerAuto("StraightSafe"));
        }

        public Command TwoMid(){
                return (new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0))).andThen
                (new AutoShooter(shooter, limelight, 44, 4000))
                                .andThen(new PathPlannerAuto("MidTaxi"))
                                .andThen(new PathPlannerAuto("Forward"))
                                .andThen(new AutoShooter(shooter, limelight, 44, 4000));
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