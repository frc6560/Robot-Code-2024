// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.team6560.frc2024.utility.ShooterConfigMap;
import com.team6560.frc2024.utility.ShooterConfigMap.Point;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int GYRO_ID = 13;

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

  // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
  // public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
  // public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
  // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(342.685546875);

  // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
  // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
  // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(258.310546875);

  // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  // public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
  // public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
  // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(137.021484375);

  // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
  // public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
  // public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;
  // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(185.2734375);

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(304.89257812499996);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(322.64648437500009);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(109.86328124999996);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(203.2031250000002);

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_ACCELERATION = 5.0; // m/s^2
  public static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(0.5, 0, 0), // Translation constants
      new PIDConstants(0.5, 0, 0), // Rotation constants
      MAX_VELOCITY_METERS_PER_SECOND,
      DRIVETRAIN_WHEELBASE_METERS, // Drive base radius (distance from center to furthest module)
      new ReplanningConfig());

  public static final int TRANSFER_MOTOR = 16;

  public static final int INTAKE_MOTOR = 15;

  public static final double TALONFX_POS_TO_ROTATION = 2048;

  public static final int LEFT_CLIMB_MOTOR = 22;

  public static final int RIGHT_CLIMB_MOTOR = 23;

  public static final double CLIMB_MAX_VERTICAL_ROTATION = 100000.0;

  public static final int CANdleID = 31;

  public static final double CLIMB_MIN_VERTICAL_ROTATION = 0.0;

  public static final double MIN_ARC_ANGLE_FOR_INTAKE = 0;

  public static final double MAX_ARC_ANGLE_FOR_INTAKE = 49.2724609375;

  public static final class ControllerIds {
    public static final int FIRST_DRIVER_CONTROLLER = 0;
    public static final int SECOND_DRIVER_CONTROL_STATION = 1;
    public static final int SECOND_DRIVER_CONTROLLER = 2;

    public static final int XBOX_L_JOY_X = 0;
    public static final int XBOX_L_JOY_Y = 1;

    public static final int XBOX_R_JOY_X = 4;
    public static final int XBOX_R_JOY_Y = 5;

    public static final int XBOX_L_BUMPER = 5;
    public static final int XBOX_R_BUMPER = 6;

    public static final int XBOX_L_TRIGGER = 2;
    public static final int XBOX_R_TRIGGER = 3;

    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_A_BUTTON = 1;

    public static final int DRIVER_STATION_TOGGLE_1 = 2;
    public static final int DRIVER_STATION_TOGGLE_2 = 5;
    public static final int DRIVER_STATION_TOGGLE_3 = 6;
    public static final int DRIVER_STATION_TOGGLE_4 = 9;

    public static final int DRIVER_STATION_BUTTON_1 = 1;
    public static final int DRIVER_STATION_BUTTON_2 = 4;
    public static final int DRIVER_STATION_BUTTON_3 = 3;

    public static final int DRIVER_STATION_X_AXIS = 0;
    public static final int DRIVER_STATION_Y_AXIS = 1;
  }

  public static final class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);

    public static final double TOP_CONE_MARKER_TO_EDGE_Z_METERS = 0.98425;
    public static final double TOP_CONE_MARKER_TO_FLOOR_DISTANCE_METERS = 1.0795;
  }

  public static final class AprilTagConstants {
    public final static double LIMELIGHT_ANGLE_DEGREES = 15.0;
    // in inches
    public final static double LIMELIGHT_HEIGHT = 10;
    public final static double ID_3_HEIGHT = 51.875, ID_4_HEIGHT = 51.875, ID_7_HEIGHT = 51.875, ID_8_HEIGHT = 51.875;
  }

  public static final class ShooterConstants {
    public static final double ACCEPTABLE_RPM_DIFF = 20;
    public static final double ACCEPTABLE_ANGLE_DIFF = 1;

    public static final int SHOOTER_LEFT_ID = 17;
    public static final int SHOOTER_RIGHT_ID = 18;

    public static final int ARC_MOTOR_ID = 14;

    public static final double RPM_TO_RPS = 1 / 60.0;

    public static final double SHOOTER_GEAR_RATIO = 30.0 / 36.0;

    public static final double ARC_GEAR_RATIO = 189.583;
    public static final double RPM_PER_FALCON_UNIT = 600.0 / 2480.0;

    public static final double MAX_ARC_ANGLE = 60;
  }

  public static final class ShooterConfigs {
    public static ShooterConfigMap shooterMap = new ShooterConfigMap();

    static {
      shooterMap.add(
          // new Point(-100.0, 6100, 17),

          // new Point(-3.19, 6100, 17),
          // new Point(-1.93, 6100, 16),
          // new Point(-0.345, 6100, 14),
          // new Point(1.29, 6100, 16.5),
          // new Point(2.73, 6000, 19),
          // new Point(4.42, 6000, 21),
          // new Point(6.76, 6000, 25),
          // new Point(9.53, 6000, 30),
          // new Point(13.47, 6000, 35),
          // new Point(18.70, 5800, 40),
          // new Point(20.0, 5500, 45),
          new Point(-100.0, 5000, 16),

          new Point(1.00, 4750, 15),
          new Point(2.03, 4550, 16),
          new Point(2.97, 4200, 18),
          new Point(4.00, 4200, 20),
          new Point(4.99, 4200, 20),
          new Point(9.05, 4000, 27),
          new Point(12.02, 4000, 30),
          new Point(14.5, 4000, 35),
          new Point(20, 4000, 44),

          // new Point(100.0, 5500, 45));
          new Point(100.0, 4000, 45));
    }

    public static final double STINGER_TRANSFER_ANGLE = 10.0; // placeholder values
    public static final double CLIMB_ANGLE = 90.0; // placeholder values
  }

  public static final class StingerConstants {
    public static final int STINGER_ELEVATOR_ID = 19;
    public static final int STINGER_WRIST_ID = 20;
    public static final int STINGER_ROLLERS_ID = 21;

    public static final double ELEVATOR_kP = 0.1;
    public static final double ELEVATOR_kD = 0.0;
    public static final double ELEVATOR_kI = 0.0001;

    public static final double WRIST_kP = 0.1;
    public static final double WRIST_kD = 0.0;
    public static final double WRIST_kI = 0.0001;
    public static final double WRIST_kFF = 0.1;

    public static final int WRIST_ENCODER_PORT = 9;
    public static final double WRIST_GEAR_RATIO = 7.0;

    public static final double TRAP_CLEARANCE_ANGLE = -25;
    public static final double STINGER_ANGLE_ACCEPTABLE_DIFF = 0.3;
    public static final double STINGER_ELEVATOR_POS_ACCEPTABLE_DIFF = 0.5;

    public static final double MIN_ELEVATOR_POS = 0.4;
    public static final double MAX_ELEVATOR_POS = 9;

    public static final double MIN_WRIST_POS = 12;
    public static final double MAX_WRIST_POS = -90;

  }

  public enum StingerConfigs {
    // add position here
    STOW(0.0, 0.0, 0.0),
    HUMAN_STATION_INTAKE(8.0, 10.0, 50.0),
    AMP_OUTTAKE(4.0, -40.0, 50.0), //placeholder values
    TRAP(8.0,-50.0,10.0), //placeholder values
    SHOOTER_TRANSFER(8.0, -80.0, 10.0);


    private double elevatorPos;
    private double stingerAngle;
    private double shooterAngle;

    private StingerConfigs(double elevatorPos, double stingerAngle, double shooterAngle) {
      this.elevatorPos = elevatorPos;
      this.stingerAngle = stingerAngle;
      this.shooterAngle = shooterAngle;
    }

    public double getElevatorPos() {
      return elevatorPos;
    }

    public double getStingerAngle() {
      return stingerAngle;
    }

    public double getShooterAngle() {
      return shooterAngle;
    }
  }

  public enum ClimbConfigs {
    // add position here
    CLIMB_STOW(0), // placeholder values
    CLIMB_RETRACTED(0), // placeholder values
    CLIMB_EXTENDED(0); // placeholder values

    private double climbPos;

    private ClimbConfigs(double climbPos) {
      this.climbPos = climbPos;
    }

    public double getClimbPos() {
      return climbPos;
    }
  }

  public static final class ClimbConstants {
    public static final double CLIMB_ACCEPTABLE_DIFF = 1;
  }

  public enum CandleColorModes {
    INTAKE_MODE, HOLD_UNREADY_MODE, SHOOT_MODE, NO_MODE, HUMAN_STATION_MODE;

    CandleColorModes() {
    };
  }

}