package com.team6560.frc2024.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private final Limelight limelight;
    private final Shooter shooter;
    private final Drivetrain drivetrain;

    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotationX();

        boolean driveResetYaw();

        boolean driveResetGlobalPose();

        boolean getAutoTarget();

        boolean autoAlignClimb();
    }

    private Controls controls;

    NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
    NetworkTableEntry ntTest = ntTable.getEntry("Initializing");
    NetworkTableEntry ntExec = ntTable.getEntry("Excecuting");
    private Command command = null;
    private boolean commandInitialized = false;

    

    public DriveCommand(Drivetrain drivetrainSubsystem, Shooter shooter, Limelight limelight, ManualControls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.limelight = limelight;
        this.shooter = shooter;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // if (controls.autoAlignClimb()) {
        //     drivetrain.setAutoLock(true);
        //     if (command == null) {
        //         commandInitialized = false;
        //         command = drivetrain.getAutoAlignCommand();
        //     }
        //     if (!commandInitialized) {
        //         command.initialize();
        //         commandInitialized = true;
        //     } else if (command.isFinished()) {
        //         drivetrain.stopModules();
        //         command = null;
        //         commandInitialized = false;
        //         return;
        //     } else {
        //         command.execute();
        //     }
        //     return;
        // } else {
        //     commandInitialized = false;
        //     command = null;
        // }
        // drivetrain.setAutoLock(false);

        var alliance = DriverStation.getAlliance();

        if (controls.driveResetYaw()) {
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                drivetrain.zeroGyroscope(new Rotation2d(Math.PI));
            } else {
                drivetrain.zeroGyroscope();
            }
        }

        if (controls.driveResetGlobalPose()) {
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                drivetrain.resetPoseEstimator(GeometryUtil.flipFieldPose(new Pose2d()));
                drivetrain.resetOnlyOdometry(GeometryUtil.flipFieldPose(new Pose2d()));
                drivetrain.zeroGyroscope();
            } else {
                drivetrain.resetPoseEstimator(new Pose2d());
                drivetrain.resetOnlyOdometry(new Pose2d());
                drivetrain.zeroGyroscope();
            }
        }

        int dir = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) ? -1 : 1;

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX() * dir,
                        controls.driveY() * dir,
                        getRotation(),
                        drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
    }

    private double getRotation() {
        double controllerInput = controls.driveRotationX();
        double limelightInput = limelight.hasTarget() ? limelight.getHorizontalAngle() : 0;

        if (Math.abs(controllerInput) <= 0.1) {

            if (controls.getAutoTarget() && shooter.getTransferSensorTriggered()) {
                return goToDelta(limelightInput);
            } //else if (controls.autoAlignClimb()) {
                // return getAlignClimb();
            // }

        }

        return controllerInput;
    }

    private double goToDelta(double delta) {

        double llDeadband = Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF / 2; // in degrees
        double rotateSpeed = 0.25; // multiplyer for max speed

        double speed = 0;
        double p = 0.3;

        if (Math.abs(delta) < llDeadband) {
            delta = 0;
        }

        speed = delta * p * rotateSpeed;

        return -speed;

    }

    private double getAlignClimb() {
        double gyro = Math.abs(drivetrain.getRawGyroRotation().getDegrees()) + 30;
        gyro %= 60;
        gyro -= 30;

        double delta = -gyro / 2;

        return goToDelta(delta);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}