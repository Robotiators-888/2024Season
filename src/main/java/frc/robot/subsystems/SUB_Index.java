// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Index extends SubsystemBase {

  CANSparkMax indexLeft = new CANSparkMax(32, MotorType.kBrushless);
  CANSparkMax indexRight = new CANSparkMax(33, MotorType.kBrushless);

  /** Creates a new SUB_Index. */
  public SUB_Index() {
    indexLeft.setInverted(true);
    indexRight.setInverted(false);
    indexLeft.setSmartCurrentLimit(40);
    indexRight.setSmartCurrentLimit(40);
    indexLeft.enableVoltageCompensation(12);
    indexRight.enableVoltageCompensation(12);
    indexRight.follow(indexLeft, true); 
    indexLeft.burnFlash();
    indexRight.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed){
    indexLeft.set(speed);
  }
}
