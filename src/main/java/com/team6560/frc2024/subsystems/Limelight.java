// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.Arrays;
import java.util.function.Supplier;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.commands.DriveCommand.Controls;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public static interface Controls {
    int getLimelightPipeline();
  }

     private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    
  private final NetworkTableEntry ntX = networkTable.getEntry("tx");
  private final NetworkTableEntry ntY = networkTable.getEntry("ty");
  private final NetworkTableEntry ntV = networkTable.getEntry("tv");
  private final NetworkTableEntry ntA = networkTable.getEntry("ta");
  private final NetworkTableEntry ntL = networkTable.getEntry("tl");
  private final NetworkTableEntry ntcL = networkTable.getEntry("cl");
  private final NetworkTableEntry ntBotPose = networkTable.getEntry("botpose_wpiblue");
  private final NetworkTableEntry ntPipeline = networkTable.getEntry("pipeline"); 

  private final Field2d aprilTagField = new Field2d();

  // private final Field2d reflectiveTapeField = new Field2d();


  public Limelight() {

    ntDispTab("Limelight")
    .add("Horizontal Angle", this::getHorizontalAngle)
    .add("Vertical Angle", this::getVerticalAngle)
    .add("Has Target", this::hasTarget)
    .add("target ID", this::getCurrentApriltagId)
    .add("taret area", this::getTargetArea);

    SmartDashboard.putData("aprilTagField", aprilTagField);
  }


  public double getHorizontalAngle() {
    return ntX.getDouble(0.0);
  }

  public double getVerticalAngle() {
    return ntY.getDouble(0.0);
  }

  public double getTargetArea() {
    return ntA.getDouble(0.0);
  }

  public boolean hasTarget(){
    return ntV.getDouble(0.0) == 1.0;
  }
 
  public int getCurrentApriltagId() {
    return (int) networkTable.getEntry("tid").getInteger(0l);
  }


  public double getLatency() {
    // 11 additional ms is recommended for image capture latency
    // divided by 1000.0 to convert ms to s
    return (ntL.getDouble(0.0) + ntcL.getDouble(11.0))/1000.0;
  }
 
  public Pose3d getTargetPoseRobotSpace() {
    double[] limelightBotPoseArray = networkTable.getEntry("targetpose_robotspace").getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    if (limelightBotPoseArray == null || limelightBotPoseArray.length < 6) return null;

    if (new double[] {0.0, 0.0, 0.0, 0.0, 0.0}.equals(Arrays.copyOf(limelightBotPoseArray, limelightBotPoseArray.length - 1)))
      return null;
    
    return new Pose3d(new Translation3d(limelightBotPoseArray[0], limelightBotPoseArray[1], limelightBotPoseArray[2]), new Rotation3d(Math.toRadians(limelightBotPoseArray[3]), Math.toRadians(limelightBotPoseArray[4]), Math.toRadians(limelightBotPoseArray[5])));

  }

  public Pair<Pose2d, Double> getBotPose() {

    if (ntPipeline.getInteger(0l) != 0 && ntPipeline.getInteger(0l) != 1) return null;
    if (!hasTarget()) return null;

    double currentTime = Timer.getFPGATimestamp() - getLatency();
    
    double[] limelightBotPoseArray = ntBotPose.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    if (limelightBotPoseArray == null || limelightBotPoseArray.length < 6) return null;

    if (new double[] {0.0, 0.0, 0.0, 0.0, 0.0}.equals(Arrays.copyOf(limelightBotPoseArray, limelightBotPoseArray.length - 1)))
      return null;
    
    Pose2d pose = new Pose3d(new Translation3d(limelightBotPoseArray[0], limelightBotPoseArray[1], limelightBotPoseArray[2]), new Rotation3d(Math.toRadians(limelightBotPoseArray[3]), Math.toRadians(limelightBotPoseArray[4]), Math.toRadians(limelightBotPoseArray[5]))).toPose2d();
    
    if (pose == null) return null;

    if (pose.equals(new Pose2d()))
      return null;
    // System.out.println("LL Field2d");
    // System.out.println(pose);

    aprilTagField.setRobotPose(pose);
    
    return new Pair<Pose2d, Double> (pose, currentTime);
  }


}