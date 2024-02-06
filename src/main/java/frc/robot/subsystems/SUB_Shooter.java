// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SUB_Shooter extends SubsystemBase {
  CANSparkMax shooterLeft = new CANSparkMax(30, MotorType.kBrushed);
  CANSparkMax shooterRight = new CANSparkMax(31, MotorType.kBrushed);
  private SparkPIDController PIDController = shooterLeft.getPIDController();
  public int MANUAL_RPM = 0;

  public void setMotorSpeed(double speed){
    shooterLeft.set(speed);
  }

  public SUB_Shooter(){
    shooterRight.setInverted(false);
    shooterRight.follow(shooterLeft, true); 
    PIDController.setOutputRange(-1, 1);
    setPIDF(PIDController, 0, 0, 0, 12/5800);
  }
  public void setPIDF(SparkPIDController pid, double P, double I, double D, double F){
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }

  public double getFlywheelRPM(){
    return shooterLeft.getEncoder().getVelocity();
  }

  public void shootFlywheelOnRPM(double rpm) {
    PIDController.setReference(rpm, ControlType.kVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
