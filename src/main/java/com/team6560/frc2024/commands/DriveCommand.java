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
    private Command goToPoseAutoCommand = null;
    private boolean goingToPose = false;
    private boolean autoAlignReady = false;
    private static ArrayList<Pose2d> trapLocations = new ArrayList<Pose2d>();

    NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
    NetworkTableEntry ntTest = ntTable.getEntry("Initializing");
    NetworkTableEntry ntExec = ntTable.getEntry("Excecuting");


    static {
        double l = 1; // meters
        // blue
        trapLocations.add(new Pose2d(Units.inchesToMeters(209.48) + l*Math.cos(180.0), 161.62 + l*Math.sin(180.0), Rotation2d.fromDegrees(180.0))); // id 14
        trapLocations.add(new Pose2d(Units.inchesToMeters(182.73) + l*Math.cos(-60.0), 177.10 + l*Math.sin(-60.0), Rotation2d.fromDegrees(-60.0))); // id 15
        trapLocations.add(new Pose2d(Units.inchesToMeters(182.73) + l*Math.cos(60.0), 146.19 + l*Math.sin(60.0), Rotation2d.fromDegrees(60.0))); // id 16

        //red
        trapLocations.add(new Pose2d(Units.inchesToMeters(468.69) + l*Math.cos(120.0), 146.19 + l*Math.sin(120.0), Rotation2d.fromDegrees(120.0))); // id 11
        trapLocations.add(new Pose2d(Units.inchesToMeters(468.69) + l*Math.cos(-120.0), 177.10 + l*Math.sin(-120.0), Rotation2d.fromDegrees(-120.0))); // id 12
        trapLocations.add(new Pose2d(Units.inchesToMeters(441.74) + l*Math.cos(0.0), 161.62 + l*Math.sin(0.0), Rotation2d.fromDegrees(0.0))); // id 13

    }

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

        if (controls.autoAlignClimb()) {
            if (goToPoseAutoCommand == null) {
                autoAlign();
                return;
            }
            
            
            
            if (!goingToPose) {
                drivetrain.setAutoLock(true);
                goToPoseAutoCommand.initialize();
                ntTest.setBoolean(true);                
                ntExec.setBoolean(false);

                System.out.println("TEST!!!");

                goingToPose = true;
            } else if (autoAlignReady) {
                goToPoseAutoCommand.execute();
                System.out.println("executing");
                
                ntTest.setBoolean(false);
                ntExec.setBoolean(true);
            } else {
                ntExec.setBoolean(false);
                ntTest.setBoolean(false);

            }

            if (goToPoseAutoCommand.isFinished()) {
                this.autoAlignReady = false;
                goToPoseAutoCommand = null;
                goingToPose = false;
                drivetrain.setAutoLock(false);
                drivetrain.stopModules();
            }

            return;
        } 
        else {
            this.autoAlignReady = true;
            drivetrain.setAutoLock(false);
            goToPoseAutoCommand = null;
            goingToPose = false;
        }

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

    private double getRotation(){
        double controllerInput = controls.driveRotationX();
        double limelightInput = limelight.hasTarget() ? limelight.getHorizontalAngle() : 0;
        
        if(Math.abs(controllerInput) <= 0.1){
                
            if (controls.getAutoTarget() && shooter.getTransferSensorTriggered()){ 
                return goToDelta(limelightInput);
            } else if(controls.autoAlignClimb()){
                return getAlignClimb();
            }
        
        }

        return controllerInput;
    }

    public void autoAlign() {
        Pose2d estimatedGlobalPose = drivetrain.getPose();
        


        Pose2d targetPose = estimatedGlobalPose.nearest(trapLocations);

        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        this.goToPoseAutoCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );

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

    private double getAlignClimb(){
        double gyro = Math.abs(drivetrain.getRawGyroRotation().getDegrees()) + 30;
        gyro %= 60;
        gyro -= 30;

        double delta = - gyro / 2;

        return goToDelta(delta);
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}