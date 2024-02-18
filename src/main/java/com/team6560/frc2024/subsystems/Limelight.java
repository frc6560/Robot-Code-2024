// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.Arrays;

import com.team6560.frc2024.Constants.AprilTagConstants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  // public static interface Controls {
  //   int getLimelightPipeline();
  // }

  private final double[] DEFAULT_DOUBLE_ARRAY = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    
  private final NetworkTableEntry ntX = networkTable.getEntry("tx");
  private final NetworkTableEntry ntY = networkTable.getEntry("ty");
  private final NetworkTableEntry ntV = networkTable.getEntry("tv");
  private final NetworkTableEntry ntA = networkTable.getEntry("ta");
  private final NetworkTableEntry ntL = networkTable.getEntry("tl");
  private final NetworkTableEntry ntcL = networkTable.getEntry("cl");
  private final NetworkTableEntry ntID = networkTable.getEntry("tid");
  private final NetworkTableEntry ntBotPose = networkTable.getEntry("botpose_wpiblue");
  private final NetworkTableEntry ntTargetPose = networkTable.getEntry("targetpose_robotspace");
  private final NetworkTableEntry ntPipeline = networkTable.getEntry("pipeline");

  private final Field2d aprilTagField = new Field2d();

  // private final Field2d reflectiveTapeField = new Field2d();

  // private final Controls controls;
  // private boolean forceOff = true;

  public Limelight(
    //Controls controls
    ) {
    //this.controls = controls;

    ntDispTab("Limelight")
    .add("Horizontal Angle", this::getHorizontalAngle)
    .add("Vertical Angle", this::getVerticalAngle)
    .add("Has Target", this::hasTarget);

    SmartDashboard.putData("aprilTagField", aprilTagField);

  }


  // public int getPipeline() {
  //   return controls.getLimelightPipeline();
  // }

  public double getHorizontalAngle() {
    return ntX.getDouble(0.0);
  }

  public double getTargetArea() {
    return ntA.getDouble(0.0);
  }

  public double getVerticalAngle() {
    return ntY.getDouble(0.0);
  }

  public boolean hasTarget(){
    return ntV.getDouble(0.0) == 1.0;
  }

  public double getLatency() {
    // 11 additional ms is recommended for image capture latency
    // divided by 1000.0 to convert ms to s
    return (ntL.getDouble(0.0) + ntcL.getDouble(11.0))/1000.0;
  }

  public double getID() {
    return ntID.getDouble(0.0);
  }

  public double getSpeakerDistance() {
    if (!hasTarget())
      return 0.0;
    
    int id = (int) getID();
    double heightDifference = 0;

    switch (id) {
      case 3: heightDifference = (AprilTagConstants.ID_3_HEIGHT - AprilTagConstants.LIMELIGHT_HEIGHT);
      case 4: heightDifference = (AprilTagConstants.ID_4_HEIGHT - AprilTagConstants.LIMELIGHT_HEIGHT);
      case 7: heightDifference = (AprilTagConstants.ID_7_HEIGHT - AprilTagConstants.LIMELIGHT_HEIGHT);
      case 8: heightDifference = (AprilTagConstants.ID_8_HEIGHT - AprilTagConstants.LIMELIGHT_HEIGHT);
    }


    return heightDifference/(Math.tan((AprilTagConstants.LIMELIGHT_ANGLE_DEGREES + getVerticalAngle()) * Math.PI/180.0));
  }

  public Pose2d getTargetPose() {
    if (ntPipeline.getInteger(0l) != 0 && ntPipeline.getInteger(0l) != 1) return null;
    if (!hasTarget()) return null;

    double[] limelightTargetPoseArray = ntTargetPose.getDoubleArray(DEFAULT_DOUBLE_ARRAY);

    if (limelightTargetPoseArray == null || limelightTargetPoseArray.length < 6) return null;

    if (new double[] {0.0, 0.0, 0.0, 0.0, 0.0}.equals(Arrays.copyOf(limelightTargetPoseArray, limelightTargetPoseArray.length - 1)))
      return null;
    
    Pose2d pose = new Pose3d(new Translation3d(limelightTargetPoseArray[0], limelightTargetPoseArray[1], limelightTargetPoseArray[2]), new Rotation3d(Math.toRadians(limelightTargetPoseArray[3]), Math.toRadians(limelightTargetPoseArray[4]), Math.toRadians(limelightTargetPoseArray[5]))).toPose2d();
    
    if (pose == null) return null;

    if (pose.equals(new Pose2d()))
      return null;

    return pose;
  }

  public Pair<Pose2d, Double> getBotPose() {

    if (ntPipeline.getInteger(0l) != 0 && ntPipeline.getInteger(0l) != 1) return null;
    if (!hasTarget()) return null;

    double currentTime = Timer.getFPGATimestamp() - getLatency();
    
    double[] limelightBotPoseArray = ntBotPose.getDoubleArray(DEFAULT_DOUBLE_ARRAY);

    if (limelightBotPoseArray == null || limelightBotPoseArray.length < 6) return null;

    if (new double[] {0.0, 0.0, 0.0, 0.0, 0.0}.equals(Arrays.copyOf(limelightBotPoseArray, limelightBotPoseArray.length - 1)))
      return null;
    
    Pose2d pose = new Pose3d(new Translation3d(limelightBotPoseArray[0], limelightBotPoseArray[1], limelightBotPoseArray[2]), new Rotation3d(Math.toRadians(limelightBotPoseArray[3]), Math.toRadians(limelightBotPoseArray[4]), Math.toRadians(limelightBotPoseArray[5]))).toPose2d();
    
    if (pose == null) return null;

    if (pose.equals(new Pose2d()))
      return null;
    //transform pose from LL "field space" to pose2d
    // pose = new Pose2d(pose.getTranslation().plus(new Translation2d(Constants.FieldConstants.length/2.0, Constants.FieldConstants.width/2.0)), pose.getRotation());

    // System.out.println("LL Field2d");
    // System.out.println(pose);

    aprilTagField.setRobotPose(pose);
    
    return new Pair<Pose2d, Double> (pose, currentTime);
  }
}
