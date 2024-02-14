// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;

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

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(342.685546875);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(258.310546875);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(137.021484375);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(185.2734375);

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
    DRIVETRAIN_WHEELBASE_METERS,// Drive base radius (distance from center to furthest module) 
    new ReplanningConfig()
  );



public static final int INTAKE_FEED_MOTOR = 15;

public static final double INTAKE_FEED_RATE = 0.6;

public static final double INTAKE_REVERSE_RATE = -0.3;




public static final double LIMELIGHT_ANGLE = 45;

public static final double LIMELIGHT_HEIGHT = 24;

public static final double APRILTAG_7_HEIGHT = 48;





public static final int ARC_MOTOR = 14;

public static final int SHOOTER_RIGHT_MOTOR = 18;

public static final int SHOOTER_LEFT_MOTOR = 17;

public static final int SHOOTER_FEED_MOTOR = 16;

public static final Port SHOOTER_COLOR_SENSOR_ID = null;

public static final double TRANSFER_FEED_OUTPUT = 1;



public static final double SHOOTER_ARC_GEAR_RATIO = 189.583;


public static final double SHOOTER_ACCEPTABLE_RPM_DIFF = 100;

public static final double SHOOTER_ACCEPTABLE_ARC_DIFF = 1;

public static final double SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF = 1;

public static final int SENSOR_TRIGGER_PROXIMITY_VALUE = 2000;




public static final int TRAP_WRIST_MOTOR = 20;

public static final int TRAP_ELEVATOR_MOTOR = 19;

public static final int TRAP_FEED_MOTOR = 21;

public static final Port TRAP_COLOR_SENSOR_ID = null;


public static final double TRAP_WRIST_GEAR_RATIO = 01;

public static final double TRAP_ELEVATOR_GEAR_RATIO = 01;

public static final double TRAP_CLEARANCE_ANGLE = 0;

public static final double TRAP_ELEVATOR_UPPER_LIMIT = 8.6;

public static final double TRAP_ELEVATOR_LOWER_LIMIT = 0;

public static final double TRAP_WALL_EXTENTION = 8.1;

public static final double TRAP_HANDOFF_EXTENTION = 4.1;




public static final double TRANSFER_INTAKE_OUTPUT = 0.8;


public static final double SHOOTER_GROUND_INTAKE_ANGLE = 0;

public static final double SHOOTER_WALL_INTAKE_ANGLE = 66.5;

public static final double SUB_SHOOTER_ANGLE = 100;

public static final double SHOOTER_SUB_RPM = 5000;

public static final double SHOOTER_SUB_ANGLE = 0;

public static final double SHOOTER_MIN_POS = 0;

public static final double SHOOTER_MAX_POS = 107;



public static final double FALCON_MAX_RPM = 6300;

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

}