package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  final CANSparkMax feedMotor;  

  public Intake() {
    this.feedMotor = new CANSparkMax(Constants.INTAKE_FEED_MOTOR, MotorType.kBrushless);
  }

  public void setIntakeFeed(boolean state){
    if(state){
      feedMotor.set(Constants.INTAKE_FEED_RATE);
    } else {
      feedMotor.set(0);
    }
  }
}
