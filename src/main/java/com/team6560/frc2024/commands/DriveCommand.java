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
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotationX();

        double driveRotationY();

        boolean driveResetYaw();

        boolean driveResetGlobalPose();

        boolean getAutoTarget();
    }

    private Controls controls;

    public DriveCommand(Drivetrain drivetrainSubsystem, Limelight limelight, Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.limelight = limelight;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
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
                    drivetrain.resetOdometry(GeometryUtil.flipFieldPose(new Pose2d()));
            } else {
                drivetrain.resetOdometry(new Pose2d());
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

    private double getRotation(){
        double controllerInput = controls.driveRotationX();

        double limelightInput = limelight.getHorizontalAngle();
        double llDeadband = 1;
        double rotateSpeed = 0.3;

        if (controllerInput == 0 && controls.getAutoTarget()){
            double speed = 0;
            double p = 0.3;

            if(Math.abs(limelightInput) < llDeadband){
                limelightInput = 0;
            }

            speed = -limelightInput * p * rotateSpeed;
            System.out.println(speed);
            return speed;
        }

        return controllerInput;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}