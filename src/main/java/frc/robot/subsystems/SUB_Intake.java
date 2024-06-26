// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class SUB_Intake extends SubsystemBase {

  public static SUB_Intake INSTANCE;
  CANSparkMax intakeMotor;
  Boolean intakeBool;
  boolean hasNote;

  /** Creates a new SUB_Intake. */
  private SUB_Intake() {

    intakeMotor = new CANSparkMax(Intake.kINTAKE_MOTOR_CANID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setInverted(true);
    Timer.delay(.1);

    intakeMotor.burnFlash();
    intakeBool = false;

    // intakeMotor.setSmartCurrentLimit(60);
    intakeMotor.setSmartCurrentLimit(60);
    // intakeMotor.getPIDController().setP(0.0);
    // intakeMotor.getPIDController().setI(0.0);
    // intakeMotor.getPIDController().setD(0.0);
    // intakeMotor.getPIDController().setFF(1.0/5800.0);
    // Timer.delay(0.2);

    intakeMotor.burnFlash();
  }

  public static SUB_Intake getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Intake();
    }

    return INSTANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/RPM", intakeMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber(getName(), 0)
    SmartDashboard.putNumber("Intake/Speed (m/sec)",
        (((intakeMotor.getEncoder().getVelocity() * 2 * Math.PI) / 8)) / 60);

  }

  /**
   * Set Speed of intake
   * 
   * @param speed Percent speed of motor
   */
  public void setMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public boolean intakeHasNote() {
    return hasNote;
  }

  public void setHasNote(boolean n) {
    hasNote = n;
  }
}
