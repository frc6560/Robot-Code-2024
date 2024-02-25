package com.team6560.frc2024.commands;

import com.pathplanner.lib.util.GeometryUtil;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private final Drivetrain drivetrain;

    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotationX();

        double driveRotationY();

        boolean driveResetYaw();

        boolean driveResetGlobalPose();

        boolean isAutoAimOn();
    }

    private Controls controls;

    private Limelight limelight;

    public DriveCommand(Drivetrain drivetrainSubsystem, Limelight limelight, Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.controls = controls;
        this.limelight = limelight;

        addRequirements(drivetrainSubsystem);
    }

    public double drivetrainAutoAim() {
        double horizontalDelta = limelight.getHorizontalAngle();
        double p = 0.1;
        double friction = 0.02;
        double deadband = 0.7;
        double rotation = controls.driveRotationX();

        if (rotation == 0 && limelight.hasTarget() && controls.isAutoAimOn()) {
            if (Math.abs(horizontalDelta) > deadband) {
                rotation = -horizontalDelta * p + friction;
            } 
        }
        return rotation;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var alliance = DriverStation.getAlliance();

        if (controls.driveResetYaw()) {
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    drivetrain.zeroGyroscope(new Rotation2d(Math.PI));
                } else {
                    drivetrain.zeroGyroscope();
                }
            } else {
                drivetrain.zeroGyroscope();
            }
        }

        if (controls.driveResetGlobalPose()) {
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    drivetrain.resetOdometry(GeometryUtil.flipFieldPose(new Pose2d()));
                } else {
                    drivetrain.resetOdometry(new Pose2d());
                }
            } else {
                drivetrain.resetOdometry(new Pose2d());
            }
        }

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -controls.driveX(),
                        -controls.driveY(),
                        controls.driveRotationX(),
                        drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
            } else {
                drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        controls.driveY(),
                        controls.driveRotationX(),
                        drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
            }
        } else {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        controls.driveY(),
                        controls.driveRotationX(),
                        drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}