// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class SUB_Intake extends SubsystemBase {

  CANSparkMax intakeMotor = new CANSparkMax(Intake.kINTAKE_MOTOR_CANID, MotorType.kBrushless);
  Boolean intakeBool;
  

  /** Creates a new SUB_Intake. */
  public SUB_Intake() {
    intakeBool = false;

    intakeMotor.getPIDController().setP(0.15);
    intakeMotor.getPIDController().setI(0);
     intakeMotor.getPIDController().setD(0.005);
    
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", intakeMotor.getEncoder().getVelocity());
  }

  /**
   * Set Speed of intake 
   *  @param speed Percent speed of motor
   */
  public void setMotorSpeed(double speed){
    //intakeMotor.set(speed);
    intakeMotor.getPIDController().setReference(speed*Constants.Intake.kOutakeRPM, ControlType.kVelocity);
  }
}
