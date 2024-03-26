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
                    drivetrain.zeroGyroscope();
            } else {
                drivetrain.resetOdometry(new Pose2d());
                    drivetrain.zeroGyroscope();
            }
        }


        // int dir = Constants.isRed() ? -1 : 1;
        
        drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                getAutoDrive().getX(),
                getAutoDrive().getY(),
                getAutoDrive().getRotation().getRadians(),
                drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
    }

    private Pose2d getAutoDrive(){
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

        if(Math.abs(rotInput) < 0.1){
            if(controls.getAutoAlignClimb()){
                // System.out.println("trying to align");
                rot = getAlignClimb();
                System.out.println(rot);

                if(rot == 0 && limelight.hasTarget()){
                    System.out.println("trying to move");
                    double m = goToDelta(limelightInput);
                    double theta = drivetrain.getRawGyroRotation().getRadians();

                    System.out.println("magnitude " + m);
                    System.out.println("Theta " + theta);

                    y = m * Math.cos(theta);
                    x = m * Math.sin(theta);


                    System.out.println("x, y: " + x + " " + y);




                } else {
                    x = 0;
                    y = 0;
                }

            } else if(controls.getAutoTarget() && shooter.getTransferSensorTriggered()){ 
                rot = goToDelta(limelightInput);
            }
            
        }
        
        return new Pose2d(x, y ,new Rotation2d(rot));
    }

    // private double getRotation(){
    //     double controllerInput = controls.driveRotationX();
    //     double limelightInput = limelight.hasTarget() ? limelight.getHorizontalAngle() : 0;
        
    //     if(Math.abs(controllerInput) <= 0.1){
                
    //         if (controls.getAutoTarget() && shooter.getTransferSensorTriggered()){ 
    //             return goToDelta(limelightInput);
    //         } else if(controls.getAutoAlignClimb()){
    //             return getAlignClimb();
    //         }
        
    //     }

    //     return controllerInput;
    // } 

    private double getAlignClimb(){
        double gyro = Math.abs(drivetrain.getRawGyroRotation().getDegrees()) + 30;
        gyro %= 60;
        gyro -= 30;

        double delta = gyro / 2;

        System.out.println("delta: " + delta);

        if (Math.abs(delta) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF) {
            return 0;
        }

        return goToDelta(delta);
        
    }

    private double goToDelta(double delta){

        double llDeadband = Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF/2; // in degrees
        double rotateSpeed = 0.25; // multiplyer for max speed

        double speed = 0;
        double p = 0.3;

        if(Math.abs(delta) < llDeadband){
            delta = 0;
        }

        speed = delta * p * rotateSpeed;

        return -speed;

    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}