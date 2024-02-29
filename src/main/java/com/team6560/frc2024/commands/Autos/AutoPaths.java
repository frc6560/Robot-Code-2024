// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.commands.Autos;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team6560.frc2024.subsystems.Intake;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class AutoPaths {
    public AutoPaths(Intake intake, Shooter shooter){
        NamedCommands.registerCommand("Intake", new AutoIntake(intake, shooter));
    }

    public static Command oneBall(Shooter shooter, Limelight limelight, Intake intake){
        Command combo = new AutoShooter(shooter, limelight, 45, 5000);
        return combo;
    }
    public static Command twoBall(Shooter shooter, Limelight limelight, Intake intake){
        Command combo = (new PathPlannerAuto("Zero")).andThen(new AutoShooter(shooter, limelight, 22, 5200))
                .andThen(new PathPlannerAuto("One")).andThen(new AutoShooter(shooter, limelight, 22 , 5200));
        return combo;
    }

    public static Command threeBall(Shooter shooter, Limelight limelight, Intake intake){
        Command combo = (new PathPlannerAuto("Zero")).andThen(new AutoShooter(shooter, limelight, 22, 5200))
                .andThen(new PathPlannerAuto("One")).andThen(new AutoShooter(shooter, limelight, 22 , 5200))
                .andThen(new PathPlannerAuto("Two")).andThen(new AutoShooter(shooter, limelight, 24, 5200));
        return combo;
    }
}
