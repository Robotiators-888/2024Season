// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Index extends SubsystemBase {

  CANSparkMax indexLeft = new CANSparkMax(32, MotorType.kBrushless);
  CANSparkMax indexRight = new CANSparkMax(33, MotorType.kBrushless);
  DigitalInput dio9 = new DigitalInput(9);

  public boolean bannersensor(){
    return dio9.get();
  }

  /** Creates a new SUB_Index. */
  public SUB_Index() {
    indexLeft.setInverted(true);
    indexRight.setInverted(false);
    indexRight.follow(indexLeft, true); 
    indexLeft.setIdleMode(IdleMode.kBrake);
    indexRight.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Index RPM", indexLeft.getEncoder().getVelocity());
  }

  public void setMotorSpeed(double speed){
    indexLeft.set(speed);
  }
}
