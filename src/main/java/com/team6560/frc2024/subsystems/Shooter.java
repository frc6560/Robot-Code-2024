// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.ArrayList;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  public class AimTrajectory implements Interpolatable<AimTrajectory>{
    double rpm;
    double angle;

    double distance;


    public AimTrajectory(double distance, double rpm, double angle){
      this.rpm = rpm;
      this.angle = angle;
      this.distance = distance;
    }

    public double getDistance(){
      return distance;
    }
    public double getAngle() {
        return angle;
    }
    public double getRpm() {
        return rpm;
    }

    @Override
    public AimTrajectory interpolate(AimTrajectory endValue, double t) {
      double angle = getAngle() + (endValue.getAngle() - getAngle()) * t;
      double rpm = getRpm() + (endValue.getRpm() - getRpm()) * t;
      double distance = getDistance() + (endValue.getDistance() - getDistance()) * t;

      return new AimTrajectory(distance, rpm, angle);
    }

    public String toString(){
      return "Distance: " + getDistance() + ", Angle: " + getAngle() + ", RPM: " + getRpm();
    }
  }
  
  DigitalInput limitSwitchSensor = new DigitalInput(Constants.SHOOTER_PROXIMITY_SENSOR_PORT);
  
  final Limelight limelight;
  final Trap trap;

  final TalonFX arcMotor;
  final TalonFX shooterMotorRight;
  final TalonFX shooterMotorLeft;

  final CANSparkMax feedMotor;

  // final ColorSensorV3 colorSensor;

  double targetRPM = 0;
  double targetPosition = 0;

  int cc = 0;
  int freq = 3;

  double sensorValue = 0;
  

  ArrayList<AimTrajectory> aimMap = new ArrayList<>();

  NetworkTable nttable = NetworkTableInstance.getDefault().getTable("ShooterPID");
  NetworkTableEntry ntP = nttable.getEntry("P");
  NetworkTableEntry ntI = nttable.getEntry("I");
  NetworkTableEntry ntD = nttable.getEntry("D");
  NetworkTableEntry ntS = nttable.getEntry("S");
  NetworkTableEntry ntV = nttable.getEntry("V");



  public Shooter(Limelight limelight, Trap trap) {
    this.limelight = limelight;
    this.trap = trap;

    arcMotor = new TalonFX(Constants.ARC_MOTOR, "Canivore");
    shooterMotorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
    shooterMotorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);

    feedMotor = new CANSparkMax(Constants.SHOOTER_FEED_MOTOR, MotorType.kBrushless);

    // colorSensor = new ColorSensorV3(Constants.SHOOTER_COLOR_SENSOR_PORT);



    setupMotors();

    
    aimMap.add(new AimTrajectory(-100.0, 5000 , 16));

    // aimMap.add(new AimTrajectory(-3.19, 6100 , 17));
    // aimMap.add(new AimTrajectory(-1.93, 6100 , 16));
    // aimMap.add(new AimTrajectory(-0.345, 6100 , 14));
    // aimMap.add(new AimTrajectory(1.29, 5400 , 17));
    // aimMap.add(new AimTrajectory(2.73, 5200 , 20    ));
    // aimMap.add(new AimTrajectory(4.42, 5200 , 24));
    // aimMap.add(new AimTrajectory(6.76, 5100 , 25));
    // aimMap.add(new AimTrajectory(9.53, 5100 , 30));
    // aimMap.add(new AimTrajectory(13.47, 5100 , 35));
    // aimMap.add(new AimTrajectory(18.70, 5100 , 40));
    // aimMap.add(new AimTrajectory(20.0, 5000 , 45));

    aimMap.add(new AimTrajectory(-1.00, 5250 , 13));

    aimMap.add(new AimTrajectory(1.00, 5250 , 15));
    aimMap.add(new AimTrajectory(2.03, 5050 , 16));
    aimMap.add(new AimTrajectory(2.97, 4700 , 18));
    aimMap.add(new AimTrajectory(3.00, 4700 , 20));
    aimMap.add(new AimTrajectory(4.99, 4250 , 22));
    aimMap.add(new AimTrajectory(7.0, 4150 , 24));
    aimMap.add(new AimTrajectory(9.0, 4100 , 28));
    aimMap.add(new AimTrajectory(11, 4000 , 30.5));
    aimMap.add(new AimTrajectory(13.57, 4000 , 33.5));
    aimMap.add(new AimTrajectory(16, 4000 , 38));
    aimMap.add(new AimTrajectory(24, 4000 , 44));
    
    aimMap.add(new AimTrajectory(100.0, 4000 , 45));



    ntDispTab("Shooter")
    .add("Shooter RPM", this::getShooterRPM)
    .add("Target RPM", ()->targetRPM)
    .add("Target Position", ()->targetPosition)
    .add("Current Draw Shooter", this::getCurrentDraw)
    .add("Vertical Position Shooter", this::getShooterArcPosition)
    .add("Transfer Triggered", this::getTransferSensorTriggered);
    // .add("Feeder Proximity Sensor", ()->colorSensor.getProximity());

    ntP.setDouble(0.0);
    ntI.setDouble(0.0);
    ntD.setDouble(0.0);
    ntS.setDouble(0.0);
    ntV.setDouble(0.0);
  }

  private void setupMotors(){
    arcMotor.setNeutralMode(NeutralModeValue.Brake);
    arcMotor.setInverted(false);

    
    // in init function
    TalonFXConfiguration arcConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    Slot0Configs arcPID = arcConfigs.Slot0;
    arcPID.kS = 0.5; // Add 0.25 V output to overcome static friction // Static Feed Forward
    arcPID.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    arcPID.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    arcPID.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    arcPID.kI = 0; // no output for integrated error
    arcPID.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output


    // set Motion Magic settings
    MotionMagicConfigs arcMM = arcConfigs.MotionMagic;  
    arcMM.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    arcMM.MotionMagicAcceleration = arcMM.MotionMagicCruiseVelocity / 0.5; // Target acceleration of 160 rps/s (0.5 seconds)
    arcMM.MotionMagicJerk = 0;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    HardwareLimitSwitchConfigs arcSwitches = new HardwareLimitSwitchConfigs();
    arcSwitches.ReverseLimitAutosetPositionEnable = true;
    arcSwitches.ReverseLimitAutosetPositionValue = 0.0;
    arcSwitches.ForwardLimitAutosetPositionEnable = true;
    arcSwitches.ForwardLimitAutosetPositionValue = 62;

    arcMotor.getConfigurator().apply(arcConfigs.withHardwareLimitSwitch(arcSwitches));
    

    

    TalonFX[] shooterMotors = new TalonFX[]{shooterMotorLeft,shooterMotorRight};

    for (TalonFX motor: shooterMotors){
      motor.setNeutralMode(NeutralModeValue.Coast);

      var talonFXConfigs = new TalonFXConfiguration();

      // set slot 0 gains
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = 0.1; // Add 0.25 V output to overcome static friction
      slot0Configs.kV = 0.11; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = 0.2; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0; // no output for integrated error
      slot0Configs.kD = 0.01; // no output for error derivative

      // set Motion Magic Velocity settings
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)




      motor.getConfigurator().apply(talonFXConfigs);
    }
    
    shooterMotorLeft.setInverted(false);
    shooterMotorRight.setInverted(true);

    feedMotor.restoreFactoryDefaults();
    feedMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    updateAtHorizontalTarget(); // running so that it updates the limelight not on target (i know its messy...)

    
    // TalonFX[] shooterMotors = new TalonFX[]{shooterMotorLeft,shooterMotorRight};

    // for (TalonFX motor: shooterMotors){
    //   // motor.setNeutralMode(NeutralModeValue.Coast);

    //   // in init function, set slot 0 gains
    //   var slot0Configs = new Slot0Configs();
    //   slot0Configs.kS = ntS.getDouble(0.0); // Add 0.05 V output to overcome static friction
    //   slot0Configs.kV = ntV.getDouble(0.0); // A velocity target of 1 rps results in 0.12 V output
    //   slot0Configs.kP = ntP.getDouble(0.0); // An error of 1 rps results in 0.11 V output
    //   slot0Configs.kI = ntI.getDouble(0.0); // no output for integrated error
    //   slot0Configs.kD = ntD.getDouble(0.0); // no output for error derivative
      
    // TalonFX[] shooterMotors = new TalonFX[]{shooterMotorLeft,shooterMotorRight};

    // for (TalonFX motor: shooterMotors){
    //   motor.setNeutralMode(NeutralModeValue.Coast);

    //   var talonFXConfigs = new TalonFXConfiguration();

    //   // set slot 0 gains
    //   var slot0Configs = talonFXConfigs.Slot0;
    //   // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    //   // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    //   // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    //   // slot0Configs.kP = 1.0; // An error of 1 rps results in 0.11 V output
    //   // slot0Configs.kI = 0; // no output for integrated error
    //   // slot0Configs.kD = 0; // no output for error derivative

    //   slot0Configs.kS = ntS.getDouble(0.0); // Add 0.05 V output to overcome static friction
    //   slot0Configs.kV = ntV.getDouble(0.0); // A velocity target of 1 rps results in 0.12 V output
    //   slot0Configs.kP = ntP.getDouble(0.0); // An error of 1 rps results in 0.11 V output
    //   slot0Configs.kI = ntI.getDouble(0.0); // no output for integrated error
    //   slot0Configs.kA = ntD.getDouble(0.0);

    //   // set Motion Magic Velocity settings
    //   var motionMagicConfigs = talonFXConfigs.MotionMagic;
    //   motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    //   motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)




    //   motor.getConfigurator().apply(talonFXConfigs);
    // }
    

    //   motor.getConfigurator().apply(slot0Configs);
    // }
    // if( cc % freq == 0){
    //   sensorValue = colorSensor.getProximity();
      
    // }
  }


  public void setRPM(double speed){
    targetRPM = speed;
    
    // if (speed > 5800) speed += 100;
    
    speed = speed / 60;



    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(speed);


    shooterMotorRight.setControl(m_request);
    shooterMotorLeft.setControl(m_request);
  }

  public void setArcPosition(double position){    
    targetPosition = position;
    
    final MotionMagicVoltage m_request = new MotionMagicVoltage(position);
    arcMotor.setControl(m_request);
  }


  public void setTransfer(double output){
    feedMotor.set(output);
  }
 


  public boolean readyToShoot(boolean isShootingSafe){
    return ( 
      Math.abs(getShooterRPM() - targetRPM) < Constants.SHOOTER_ACCEPTABLE_RPM_DIFF &&
      (isShootingSafe ||isAtTargetArcAngle()) &&
      (isShootingSafe || isAtTargetHorizontalAngle())
      // trap.isClearOfShooter()
    );
  }
  
  public AimTrajectory findClosest(double dist){
    AimTrajectory lower = new AimTrajectory(0, Constants.SHOOTER_SUBWOOFER_RPM, Constants.SHOOTER_SUBWOOFER_POSITION);
    AimTrajectory upper = new AimTrajectory(0, Constants.SHOOTER_SUBWOOFER_RPM, Constants.SHOOTER_SUBWOOFER_POSITION);
    double t;

    for(AimTrajectory trajectory : aimMap){
      // System.out.println(trajectory);
      if(trajectory.getDistance() < dist){
        lower = trajectory;
      } else {
        upper = trajectory;
        break;
      }
    }

    t = (dist - lower.getDistance()) / (upper.getDistance() - lower.getDistance());

    AimTrajectory newTrajectory = lower.interpolate(upper, t);
    // System.out.println("new traj = " + newTrajectory);
    return newTrajectory;
  }


  public boolean getTransferSensorTriggered(){
    // return sensorValue > Constants.SENSOR_TRIGGER_PROXIMITY_VALUE;
    return  ! limitSwitchSensor.get();
  }


  public double getShooterRPM(){
    return shooterMotorLeft.getRotorVelocity().getValueAsDouble() * 60;
  }

  public double getCurrentDraw(){
    return shooterMotorLeft.getTorqueCurrent().getValueAsDouble();
  }

  public double getShooterArcPosition(){
    return arcMotor.getRotorPosition().getValueAsDouble();
    // return arcMotor.getRotorPosition().getValueAsDouble();
  }

  double horizontalOnTarget = 0;

  public boolean isAtTargetHorizontalAngle(){
    return horizontalOnTarget > 9;
  }

  private void updateAtHorizontalTarget(){
    if (limelight.hasTarget() && Math.abs(limelight.getHorizontalAngle()) < Constants.SHOOTER_ACCEPTABLE_HORIZONTAL_DIFF ){
      horizontalOnTarget += 1;
    } else {
      horizontalOnTarget = 0;
    }

  }

  public boolean isAtTargetArcAngle(){
    return Math.abs(getShooterArcPosition() - targetPosition) < Constants.SHOOTER_ACCEPTABLE_ARC_DIFF;
  }

  public boolean isAtTargetArcAngle(double target){
    return Math.abs(getShooterArcPosition() - target) < Constants.SHOOTER_ACCEPTABLE_ARC_DIFF;
  }
}
