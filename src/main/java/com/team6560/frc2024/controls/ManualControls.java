// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.controls;

import com.team6560.frc2024.Constants;
// import com.team6560.frc2024.Constants.*;
import com.team6560.frc2024.commands.DriveCommand;
import com.team6560.frc2024.utility.NumberStepper;
import com.team6560.frc2024.utility.PovNumberStepper;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ManualControls implements DriveCommand.Controls {
  private XboxController xbox;

  private final PovNumberStepper speed;
  private final PovNumberStepper turnSpeed;

  private NetworkTable climbTable;

  private NetworkTable intakeTable;

  private XboxController controlStation;

  private NetworkTable armTable;

  // private boolean prevclimbEngaged;
  private boolean climbEngaged = false;

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
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  
  // ------------------------------ DRIVETRAIN ------------------------------ \\

  @Override
  public double driveX() {
    double speedModifier = nitroMode() ? 1.25 : 1;

    return -modifyAxis(xbox.getLeftY() * speed.get() * speedModifier);
  }

  @Override
  public double driveY() {
    double speedModifier = nitroMode() ? 1.25 : 1;

    return -modifyAxis(xbox.getLeftX() * speed.get() * speedModifier);
  }

  @Override
  public double driveRotationX() {
    return -modifyAxis(xbox.getRightX() * turnSpeed.get());
  }

  @Override
  public boolean driveResetYaw() {
    return xbox.getStartButton();
  }

  @Override
  public boolean driveResetGlobalPose() {
    return xbox.getBackButton();
  }

  public boolean nitroMode(){
    return xbox.getRightTriggerAxis() > 0.2;
  }


  
  // ------------------------------ SHOOTER ------------------------------ \\

  @Override
  public boolean getAutoTarget(){
    return getAim();
  }

  public boolean getAim(){
    return controlStation.getRightBumper();
    // return false;
  }
  
  public boolean getSafeAim(){
    return controlStation.getRightTriggerAxis() > 0.2;
  }

  public boolean getShoot(){
    return xbox.getRightBumper();
  }

  public boolean getReverseTransfer(){
    return controlStation.getXButton();
  }
  

  // ------------------------------ INTAKE ------------------------------ \\

  public boolean getRunIntake(){
    return getSafeAim() || getAim();
    // return controlStation.getLeftBumper();
  }

  // ------------------------------ TRAP ------------------------------ \\

  public boolean getTrapTransferIn(){
    return controlStation.getBButton();
  }

  public boolean getTrapTransferOut(){
    return false; //controlStation.getXButton();
  }
  
  public boolean getAmpPlace(){
    return controlStation.getAButton();
  }
  
  public boolean getTrapIntake(){
    return controlStation.getLeftBumper();
  }

  public boolean getTrapPlace() {
    if (controlStation.getYButtonPressed  ()) 
      climbEngaged = !climbEngaged;
      
    return climbEngaged;
  }

  public boolean getTrapPlaceOver() {
    return !getTrapPlace() && controlStation.getYButtonReleased();
  }


  

  // ------------------------------ CLIMB ------------------------------ \\

  public double getClimbLeft(){
   return deadband(controlStation.getLeftY(), 0.1);
  }
  public double getClimbRight(){
    if(controlStation.getLeftTriggerAxis() > 0.2){
      return getClimbLeft();
    }
    return deadband(controlStation.getRightY()/1.1, 0.1);
  }

  @Override
  public boolean autoAlignClimb() {
    return xbox.getAButton();
  }




  public void setRumble(double output){
    xbox.setRumble(RumbleType.kBothRumble, output);
  }
}