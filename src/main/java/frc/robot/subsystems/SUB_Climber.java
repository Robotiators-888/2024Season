package frc.robot.subsystems;

import static frc.robot.Constants.Pivot.*;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.libs.PIDGains;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Climber extends SubsystemBase {

  private SparkPIDController climberPID;
  
  CANSparkMax climberRight;
  CANSparkMax climberLeft;
  Boolean climberUp;

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

    // Initialize the PID controller
    climberPID = climberLeft.getPIDController();  
    setPIDF(climberPID, 0.013, 0, 0.001, 0);
  
  }

  public void setPIDF(SparkPIDController pid, double P, double I, double D, double F){
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }

  public void runMotor(double pwr) {
    climberLeft.set(pwr);
  }

  public void holdPosition() {
    double targetPosition = climberLeft.getEncoder().getPosition(); 
    climberPID.setReference(targetPosition, ControlType.kPosition); 
  }
}
