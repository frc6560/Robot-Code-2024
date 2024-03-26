package com.team6560.frc2024.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team6560.frc2024.Constants;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  final CANSparkMax feedMotor;

  DigitalInput proxitySensor = new DigitalInput(Constants.INTAKE_PROXIMITY_SENSOR_PORT);

  public Intake() {
    this.feedMotor = new CANSparkMax(Constants.INTAKE_FEED_MOTOR, MotorType.kBrushless);
    feedMotor.restoreFactoryDefaults();
    feedMotor.setIdleMode(IdleMode.kCoast);

    ntDispTab("Intake")
    .add("Intake Feed Speed", this::getFeedSpeed)
    .add("Intake Sensor", this::getProximitySensor)
    .add("Intake Current Draw", this::getCurrentDraw);
  }

  public void setIntakeFeed(double output){
    feedMotor.set(output);
  }

  public boolean getProximitySensor(){
    return !proxitySensor.get();
  }

  public double getFeedSpeed(){
    return feedMotor.getEncoder().getVelocity();
  }

  public double getCurrentDraw(){
    return feedMotor.getOutputCurrent();
  }
}
