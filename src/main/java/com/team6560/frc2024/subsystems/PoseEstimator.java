// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import com.team6560.frc2024.Constants;
import com.team6560.frc2024.Constants.PoseEstimatorConstants;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  private Limelight limelight;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private boolean overrideMaxVisionPoseCorrection;

  private Pose2d lastPose;

  private Supplier<Rotation2d> rawGyroSupplier;

  private Supplier<SwerveModulePosition[]> swerveModulePositionSupplier;

  private static final Pose2d EMPTY_POSE_2D = new Pose2d();

  public PoseEstimator(Supplier<Rotation2d> rawGyroSupplier,
      Supplier<SwerveModulePosition[]> swerveModulePositionSupplier, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.rawGyroSupplier = rawGyroSupplier;
    this.swerveModulePositionSupplier = swerveModulePositionSupplier;
    this.limelight = limelight;

    SmartDashboard.putData("Field", field2d);

    poseEstimator = new SwerveDrivePoseEstimator(Constants.m_kinematics,
        rawGyroSupplier.get(),
        swerveModulePositionSupplier.get(),
        new Pose2d(),
        PoseEstimatorConstants.stateStdDevs,
        PoseEstimatorConstants.visionMeasurementStdDevs);
        
  }

  @Override
  public void periodic() {
    field2d.setRobotPose(getEstimatedPosition());

    poseEstimator.update(rawGyroSupplier.get(), swerveModulePositionSupplier.get());
    updateVision();

  }

  private void updateVision() {
    Pair<Pose2d, Double> result = limelight.getBotPose();
    if (result == null)
      return;

    Pose2d camPose = result.getFirst();

    if (camPose == null || camPose.equals(EMPTY_POSE_2D))
      return;

    if (!overrideMaxVisionPoseCorrection) {
      camPose = new Pose2d(camPose.getTranslation(), getGyroscopeRotation());
    }
    if (camPose.minus(getEstimatedPosition()).getTranslation().getNorm() > 1.5 && !overrideMaxVisionPoseCorrection)
      return;

    double camPoseObsTime = result.getSecond();
    poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
  }

  public Pose2d getEstimatedPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getGyroscopeRotation() {
    if (poseEstimator == null) {
      if (lastPose == null) {
        return new Rotation2d();
      }
      return lastPose.getRotation();
    }

    return getEstimatedPosition().getRotation();
  }
}

