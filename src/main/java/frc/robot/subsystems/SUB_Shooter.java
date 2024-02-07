// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SUB_Shooter extends SubsystemBase {
  CANSparkMax shooterLeft = new CANSparkMax(30, MotorType.kBrushless);
  CANSparkMax shooterRight = new CANSparkMax(31, MotorType.kBrushless);

  public void setMotorSpeed(double speed){
    shooterLeft.set(speed);
  }

  public SUB_Shooter(){
    shooterRight.setInverted(true);
    shooterRight.follow(shooterLeft, false);
    
  }

  public void periodic(){
    SmartDashboard.putNumber("Shooter RPM", shooterLeft.getEncoder().getVelocity());
  }
}