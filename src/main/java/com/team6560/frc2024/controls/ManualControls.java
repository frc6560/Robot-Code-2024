// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.controls;

import com.team6560.frc2024.Constants;
// import com.team6560.frc2024.Constants.*;
import com.team6560.frc2024.commands.DriveCommand;
import com.team6560.frc2024.commands.IntakeCommand;
import com.team6560.frc2024.commands.ShooterCommand;
import com.team6560.frc2024.commands.ClimbCommand;
import com.team6560.frc2024.commands.StingerCommand;
import com.team6560.frc2024.commands.LightWorkNoReactionCommand;


import com.team6560.frc2024.utility.NumberStepper;
import com.team6560.frc2024.utility.PovNumberStepper;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ManualControls implements DriveCommand.Controls, IntakeCommand.Controls, ShooterCommand.Controls, StingerCommand.Controls, ClimbCommand.Controls, LightWorkNoReactionCommand.Controls {
  private XboxController xbox;

  private final PovNumberStepper speed;
  private final PovNumberStepper turnSpeed;

  // private NetworkTable limelightTable;

  private NetworkTable climbTable;

  private NetworkTable intakeTable;

  private XboxController controlStation;

  private NetworkTable armTable;

  // private boolean prevclimbEngaged;
  // private boolean climbEngaged;

  // private boolean prevIntakeOverrideEngaged;

  // private boolean intakeOverrideEngaged;

  /**
   * Creates a new `ManualControls` instance.
   *
   * @param xbox the Xbox controller to use for manual control
   */
  public ManualControls(XboxController xbox) {
    this(xbox, null);
  }

  public ManualControls(XboxController xbox, XboxController controlStation) {
    this.xbox = xbox;
    if (controlStation == null)
      this.controlStation = xbox;
    else
      this.controlStation = controlStation;

    this.speed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.4, 0.0,
            Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.6, Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.025),
        xbox,
        PovNumberStepper.PovDirection.VERTICAL);

    this.turnSpeed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.175, 0.0,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.15,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.0025),
        xbox,
        PovNumberStepper.PovDirection.HORIZONTAL);
    
    ntDispTab("Controls")
      .add("Y Joystick", this::driveY)
      .add("X Joystick", this::driveX)
      .add("Rotation Joystick", this::driveRotationX);

    
    // limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");
    intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    armTable = NetworkTableInstance.getDefault().getTable("Arm");
    intakeTable.getEntry("speed").setDouble(0.0);

    // limelightTable.getEntry("limelightPipeline").setInteger( (long) 0);
    
    climbTable = NetworkTableInstance.getDefault().getTable("Climb");

    climbTable.getEntry("isClimbing").setBoolean(false);

    climbTable.getEntry("climbVelocityL").setDouble(0.0);

    climbTable.getEntry("climbVelocityR").setDouble(0.0);

    armTable.getEntry("resetArmZero").setBoolean(false);

    armTable.getEntry("overrideSoftLimits").setBoolean(false);

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.10);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }



  /* INTAKE */
  @Override
  public boolean getIntakeIn() {
    return xbox.getRightBumper();
  }

  @Override
  public boolean getIntakeInReleased() {
    return xbox.getRightBumperReleased();
  }

  @Override
  public boolean getIntakeOut() {
    return xbox.getLeftBumper();
  }
  
  @Override
  public boolean getIntakeOutReleased() {
    return xbox.getLeftBumperReleased();
  }
  
  /* SHOOTER */
  @Override
  public boolean getManualShootShooter() {
    return xbox.getRightTriggerAxis() > 0.5;
  }

  @Override
  public double getManualAim() {
    return controlStation.getLeftY();
  }

  @Override
  public double getManualShooterSpeed() {
    return controlStation.getLeftX();
  }

  @Override
  public boolean aButtonSetManualMode() {
    return xbox.getAButtonPressed();
  }

  @Override
  public boolean setStowPos() {
    return xbox.getBButtonPressed();
  }

  @Override
  public double getClimbControls() {
    return controlStation.getRightY(); 
  }

  /* STINGER */
  @Override
  public boolean manualStow() {
    return xbox.getBButtonPressed();
  }

  @Override
  public boolean manualStingerIntakePos() {
    return controlStation.getYButtonPressed();
  }

  @Override
  public boolean manualStingerShooterTransfer() {
    return controlStation.getBButtonPressed();
  }

  @Override
  public double manualElevatorVelControl() {
    return controlStation.getLeftY();
  }

  public double manualStingerAngleControl() {
    return controlStation.getLeftX();
  }



  // private static double modifyAxis2(double value) {
  //   // Deadband
  //   value = deadband(value, 0.1);

  //   // Square the axis
  //   value = Math.copySign(value * value, value);

  //   return value;
  // }

  /**
   * Returns the x component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the x component of the robot's velocity
   */
  @Override
  public double driveX() {
    return - modifyAxis(xbox.getLeftY() * speed.get());
  }

  /**
   * Returns the y component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the y component of the robot's velocity
   */
  @Override
  public double driveY() {
    return - modifyAxis(xbox.getLeftX() * speed.get());
  }

  /**
   * Returns the angular velocity of the robot, as controlled by the Xbox
   * controller.
   *
   * @return the angular velocity of the robot
   */
  @Override
  public double driveRotationX() {
    return modifyAxis(xbox.getRightX() * turnSpeed.get());
  }

  @Override
  public double driveRotationY() {
    return modifyAxis(xbox.getRightY() * turnSpeed.get());
  }

  /**
   * Returns whether the yaw of the robot's gyroscope should be reset, as
   * controlled by the Xbox controller.
   *
   * @return whether the yaw of the robot's gyroscope should be reset
   */
  @Override
  public boolean driveResetYaw() {
    return xbox.getStartButton();
  }


  @Override
  public boolean driveResetGlobalPose() {
    return xbox.getBackButton();
  }

}