// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.team6560.frc2024.Constants;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.team6560.frc2024.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Lights extends SubsystemBase {
  private final CANdle candle;

  /** Creates a new Lights. */
  public Lights() {
    this.candle = new CANdle(Constants.CANDLE_ID);
    candle.configBrightnessScalar(0.7);
    
    ntDispTab("Lights")
    .add("candle current", () -> candle.getCurrent());
  }

  @Override
  public void periodic() {
    
    // candle.setLEDs((int)(targetColor.red *255),(int)(targetColor.green *255), (int)(targetColor.blue *255));
  }

  public void setColor(Color targetColor){
    // candle.clearAnimation(0);
    
    candle.setLEDs((int)(targetColor.red *255),(int)(targetColor.green *255), (int)(targetColor.blue *255));
  }

  public void setLightsIntake() {
    setColor(Color.kYellow);
    // candle.animate(new FireAnimation(0.8, 0.4, 70, 0, 0));
  }

  public void setLightsShooting(boolean shooterReady) {
    if(shooterReady){
     setColor(Color.kGreen);
    } else {
      setColor(Color.kRed);
    }
  }

  public void setLightsTrapIntake() {
    setColor(Color.kPurple);
  }

  public void setLightsTrapPlace(){
    setColor(Color.kGreen);
  }

  public void setLightsIdle() {
    setColor(Color.kBlack);
  }

  public void setLightsTrapTransfer(boolean sensorTriggered) {
    if(sensorTriggered){
      setColor(Color.kGreen);
    } else {
      setColor(Color.kRed);
    }
  }

  public void setLightsReverse() {
    setColor(Color.kDeepPink);
  }
  
  public void setLightsHasPiece(){
    // candle.animate(new RainbowAnimation());
    setColor(Color.kDarkRed);
  }
}