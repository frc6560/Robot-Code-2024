package com.team6560.frc2024.commands;

import com.pathplanner.lib.util.GeometryUtil;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.controls.ManualControls;
import com.team6560.frc2024.subsystems.Drivetrain;
import com.team6560.frc2024.subsystems.Limelight;
import com.team6560.frc2024.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Kinematics.ProtobufChassisSpeeds;
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
    }

    private ManualControls controls;

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
        if (controls.driveResetYaw()) {
            if (Constants.isRed()) {
                drivetrain.zeroGyroscope(new Rotation2d(Math.PI));
            } else {
                drivetrain.zeroGyroscope();
            }
        }

        if (controls.driveResetGlobalPose()) {
            if (Constants.isRed()) {
                    drivetrain.resetOdometry(GeometryUtil.flipFieldPose(new Pose2d()));
                    drivetrain.zeroGyroscope();
            } else {
                drivetrain.resetOdometry(new Pose2d());
                    drivetrain.zeroGyroscope();
            }
        }


        // int dir = Constants.isRed() ? -1 : 1;
        
        drivetrain.drive(
            getAutoDrive()); // perhaps use getRawGyroRotation() instead?
    }

    private ChassisSpeeds getAutoDrive(){
        double rot = 0;
        double x = 0;
        double y = 0;

        double limelightInput = limelight.hasTarget() ? limelight.getHorizontalAngle() : 0;

        int dir = Constants.isRed() ? -1 : 1;
        double xInput = controls.driveX() * dir;
        double yInput = controls.driveY() * dir;
        double rotInput = controls.driveRotationX();
        
        x = xInput;
        y = yInput;
        rot = rotInput;
        boolean align = false;

        if(Math.abs(rotInput) < 0.1){
            if(controls.getAutoAlignClimb()){
                Pose2d targetPose = drivetrain.getAutoAlignTargetPose();
                Pose2d robotPose = drivetrain.getLimelightPose();

                System.out.println();
                System.out.println("x " + robotPose.getX() + " y " + robotPose.getY() + " rot " + robotPose.getRotation().getDegrees());
                System.out.println("x " + targetPose.getX() + " y " + targetPose.getY() + " rot " + targetPose.getRotation().getDegrees());
                System.out.println();
                // System.out.println("trying to align");
                rot = getAlignClimb();
                


                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();
                
                double pidCorrection = 15;

                dx *= pidCorrection;
                dy *= pidCorrection;

                System.out.println("dx " + dx + " dy " + dy);

                x = -goToDelta(dx);
                y = -goToDelta(dy);
                System.out.println("x " + x + " y " + y);

                align = true;
                    

            } else if(controls.getAutoTarget() && shooter.getTransferSensorTriggered()){ 
                rot = goToDelta(limelightInput);
            }
            
        }

        Rotation2d robotRotation = align ? drivetrain.getLimelightPose().getRotation() : drivetrain.getGyroscopeRotationNoApriltags();
        
        ChassisSpeeds output =  ChassisSpeeds.fromFieldRelativeSpeeds(
                x,
                y,
                rot,
                robotRotation);
        
        return output;
    }

    private double getAlignClimb(){
        Rotation2d gyro = drivetrain.getLimelightPose().getRotation();
        Rotation2d targetRot = drivetrain.getAutoAlignTargetPose().getRotation();

        double delta = gyro.minus(targetRot).getDegrees();
        
        if (Math.abs(delta) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF/1.5) {
            return 0;
        }

        return goToDelta(delta);
        
    }

    private double goToDelta(double delta, double deadband){

        double llDeadband = deadband;
        double rotateSpeed = 0.25; // multiplyer for max speed

        double speed = 0;
        double p = 0.3;

        if(Math.abs(delta) < llDeadband){
            delta = 0;
        }

        speed = delta * p * rotateSpeed;

        return -speed;

    }

    private double goToDelta(double delta){
        return goToDelta(delta, Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF / 2);
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}