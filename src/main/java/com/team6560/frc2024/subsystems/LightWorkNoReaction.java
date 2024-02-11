// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.team6560.frc2024.Constants;
import com.team6560.frc2024.Constants.CandleColorModes;

public class LightWorkNoReaction extends SubsystemBase {
  private final CANdle candle; 
  private CandleColorModes mode;
  private boolean isFlashOn;
  private int isFlashOnCounter;
  /** Creates a new LightWorkNoSweat. */
  public LightWorkNoReaction() {
    this.candle = new CANdle(Constants.CANdleID);
    this.mode = CandleColorModes.NO_MODE; 
    this.isFlashOn = false;
    this.isFlashOnCounter = 0;
  }
  public CandleColorModes getColorMode() {
    return mode; 
  }

  public void setColorMode(CandleColorModes mode){
    this.mode = mode;
  }

  public void setColor(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  public void setColor(Color color) {
    candle.setLEDs((int) color.red *255, (int) color.green*255, (int) color.blue*255);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isFlashOnCounter++;
    if (isFlashOnCounter == 4) {
      isFlashOn = !isFlashOn;
      isFlashOnCounter = 0;
    }
    isFlashOn = !isFlashOn;
    if (mode == CandleColorModes.HOLD_MODE) {
      candle.setLEDs(0,17,255); // ask Alex for team's RGB for blue
    } else if (mode == CandleColorModes.INTAKE_MODE && isFlashOn) {
      candle.setLEDs(255, 93, 13); 
    } else if (mode == CandleColorModes.SHOOT_MODE) {
      candle.setLEDs((int)(Math.random()*256), (int)(Math.random()*256), (int)(Math.random()*256));
    }
  }
}
