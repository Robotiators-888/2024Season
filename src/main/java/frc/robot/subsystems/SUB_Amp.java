// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Amp extends SubsystemBase {
  static SUB_Amp INSTANCE = null;
  CANSparkMax ampController;
  Timer currentTimer = new Timer();

  public static SUB_Amp getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Amp();
    }

    return INSTANCE;
  }

  /** Creates a new SUB_Amp. */
  private SUB_Amp() {
    ampController = new CANSparkMax(42, MotorType.kBrushless);
    ampController.restoreFactoryDefaults();

    ampController.setSmartCurrentLimit(30);

    ampController.setInverted(false);
    ampController.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    ampController.set(speed);
  }
  public void setbreak(){
    ampController.setIdleMode(IdleMode.kBrake);
  }
  public void setcoast(){
    ampController.setIdleMode(IdleMode.kCoast);
  }

  public boolean isAtForwardLimit() {
    return ampController.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public boolean isAtReverseLimit() {
    return ampController.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public void starttimer() {
    currentTimer.reset();
    currentTimer.start();
  }

  public boolean CurrentLimitSpike() {
    double avg = ampController.getOutputCurrent();

    SmartDashboard.putNumber("Amp/OutputCurrent", avg);
    return (currentTimer.hasElapsed(.3) && avg > 15.0);
  }

}
