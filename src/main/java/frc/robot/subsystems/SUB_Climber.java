package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import frc.libs.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Climber extends SubsystemBase {

//weight of robot : 115
//force of gravity: 511.19860099 Neutons
//511.19/2 NEO motors: 255.595 Neutons per neo disreguarding springs
//

  CANSparkMax climberRight;
  CANSparkMax climberLeft;
  Boolean climberUp;
  private TrapezoidProfile.State targetState;
  private TrapezoidProfile.State currentState;
  private double homepos = Constants.Climber.kClimberHomePos;
  private SparkPIDController climberPID;

  

  public SUB_Climber(){
    climberRight = new CANSparkMax(36, MotorType.kBrushless);
    climberLeft = new CANSparkMax(37, MotorType.kBrushless);
    
    
    climberLeft.restoreFactoryDefaults();
    climberRight.restoreFactoryDefaults();
    climberRight.setInverted(false);
    climberRight.follow(climberLeft, false);
    climberRight.setIdleMode(IdleMode.kBrake);
    climberLeft.setIdleMode(IdleMode.kBrake);
    climberLeft.burnFlash();
    climberRight.burnFlash();

 
    climberPID = climberLeft.getPIDController();
    climberPID.setOutputRange(-0.8, 0.8);
    setPIDF(climberPID, 1.3, 0, 0.7, 1.1);

  }

  public void setPIDF(SparkPIDController pid, double P, double I, double D, double F){
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }
  
  public void goHomeG(){
    targetState = new TrapezoidProfile.State(homepos, 0.0);
    currentState = new TrapezoidProfile.State(climberLeft.getEncoder().getPosition(), climberLeft.getEncoder().getVelocity());
  }

  public void setTargetState(){
    homepos = climberLeft.getEncoder().getPosition();
  }




  public void runMotor(double pwr ) {
   climberLeft.setVoltage(pwr);
  }


}
