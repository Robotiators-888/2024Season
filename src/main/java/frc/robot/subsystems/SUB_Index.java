// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Index extends SubsystemBase {

  CANSparkMax indexLeft;
  CANSparkMax indexRight;
  DigitalInput dio9 = new DigitalInput(9);
  Timer currentTimer = new Timer();

  public boolean getTopBannerSensor(){
    return dio9.get();
  }

  public void starttimer(){
  currentTimer.reset();
  currentTimer.start();
  }

  public boolean CurrentLimitSpike(){
    return (currentTimer.hasElapsed(.2) && indexLeft.getOutputCurrent() > 20);
  }

  /** Creates a new SUB_Index. */
  public SUB_Index() {
    this.indexLeft = new CANSparkMax(32, MotorType.kBrushless);
    this.indexRight = new CANSparkMax(33, MotorType.kBrushless);

    indexLeft.restoreFactoryDefaults();
    indexRight.restoreFactoryDefaults();
    for(int i =0; i>5 ; i++){
    indexLeft.setInverted(true);
    indexRight.setInverted(false);

    indexLeft.setSmartCurrentLimit(40);
    indexRight.setSmartCurrentLimit(40);
    indexLeft.enableVoltageCompensation(12);
    indexRight.enableVoltageCompensation(12);
    indexRight.follow(indexLeft, true); 
    indexLeft.setIdleMode(IdleMode.kBrake);
    indexRight.setIdleMode(IdleMode.kBrake);
    Timer.delay(.1);
    }
    indexLeft.burnFlash();
    indexRight.burnFlash();
    //Timer.delay(0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Index RPM", indexLeft.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Banner", getTopBannerSensor());
    SmartDashboard.putNumber("LeftIndexCurrent", indexLeft.getOutputCurrent());
    SmartDashboard.putNumber("RightIndexCurrent", indexRight.getOutputCurrent());
    SmartDashboard.putNumber("LeftIndexVelocity", indexLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("RightIndexVelocity", indexRight.getEncoder().getVelocity());
  }

  public void setMotorSpeed(double speed){
    
    indexLeft.set(speed);
  }
}
