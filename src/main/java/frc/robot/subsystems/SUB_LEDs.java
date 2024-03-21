// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_LEDs extends SubsystemBase {
  /** Creates a new SUB_Leds. */
  Spark blinkin;

  public SUB_LEDs(int port) {
    blinkin = new Spark(port);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      blinkin.set(val);
    }
  }
}
