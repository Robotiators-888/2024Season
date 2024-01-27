// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SUB_Shooter extends SubsystemBase {
  CANSparkMax shooterLeft = new CANSparkMax(30, MotorType.kBrushed);
  CANSparkMax shooterRight = new CANSparkMax(31, MotorType.kBrushed);
  CANSparkMax indexLeft = new CANSparkMax(32, MotorType.kBrushless);
  CANSparkMax indexRight = new CANSparkMax(33, MotorType.kBrushless);

  public Boolean shooterBool;
  public Boolean indexBool;

   
    

  public void driveShooter(){
    if(shooterBool){
      shooterLeft.set(-.5);
    }else{
      shooterLeft.set(0);
    }
    shooterBool = !shooterBool;
  }

  public void driveIndex(){
    if(indexBool){
      indexLeft.set(.5);
    }else{
      indexLeft.set(0);
    }
    indexBool = !indexBool;
  }

  public SUB_Shooter(){
    shooterBool = false;
    indexBool = false;
    shooterRight.setInverted(false);
    shooterRight.follow(shooterLeft, true);
    indexLeft.setInverted(true);
    indexRight.setInverted(false);
    indexRight.follow(indexLeft, true); 
  }
}
