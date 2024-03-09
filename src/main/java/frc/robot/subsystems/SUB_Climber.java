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

/**
 * This class represents the Climber subsystem of the robot.
 */
public class SUB_Climber extends SubsystemBase {

  CANSparkMax climberRight;
  CANSparkMax climberLeft;
  Boolean climberUp;
  private TrapezoidProfile.State targetState;
  private TrapezoidProfile.State currentState;
  private double homepos = Constants.Climber.kClimberHomePos;
  private SparkPIDController climberPID;

  /**
   * Constructor for the SUB_Climber class.
   * Initializes the climber motors and sets their default configurations.
   */
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

  /**
   * Sets the PIDF coefficients for a SparkPIDController.
   * @param pid The SparkPIDController to set the coefficients for.
   * @param P The proportional coefficient.
   * @param I The integral coefficient.
   * @param D The derivative coefficient.
   * @param F The feedforward coefficient.
   */
  public void setPIDF(SparkPIDController pid, double P, double I, double D, double F){
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }
  
  /**
   * Sets the target state for the climber to return to its home position.
   */
  public void climberFreezePostion(){
    targetState = new TrapezoidProfile.State(homepos, 0.0);
    currentState = new TrapezoidProfile.State(climberLeft.getEncoder().getPosition(), climberLeft.getEncoder().getVelocity());
  }

  /**
   * Updates the home position to the current position of the left climber motor.
   */
  public void setTargetState(){
    homepos = climberLeft.getEncoder().getPosition();
  }

  /**
   * Runs the left climber motor at a specified power.
   * @param pwr The power to set the motor to, as a voltage.
   */
  public void runMotor(double pwr) {
   climberLeft.setVoltage(pwr);
  }
}
