package frc.robot.subsystems;

import static frc.robot.Constants.Pivot.*;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.libs.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Pivot extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    public final SparkAbsoluteEncoder rotateEncoder;
    private final RelativeEncoder rotateRelativeEncoder;
    private Timer pivotTimer;
    private TrapezoidProfile pivotTrapezoidProfile;
    private SparkPIDController pivotPID;
    private double pivotSetpoint;
    private AbsoluteEncoder pivotEncoder;
    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;


 public SUB_Pivot(){
        pivotMotor = new CANSparkMax(kPIVOT_ROTATE_MOTOR_CANID, MotorType.kBrushless);
        rotateEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        rotateRelativeEncoder = pivotMotor.getEncoder();

        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(50);
        // pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // pivotEncoder.setVelocityConversionFactor(1.0/4.0 * 2 * Math.PI);
        // pivotEncoder.setPositionConversionFactor(1.0/4.0 * 2 * Math.PI);
        // rotateRelativeEncoder.setPositionConversionFactor(1.0/(300.0)*2*Math.PI);
        // rotateRelativeEncoder.setPosition(pivotEncoder.getPosition());
        pivotMotor.burnFlash();
        pivotPID = pivotMotor.getPIDController();
        PIDGains.setSparkMaxGains(pivotPID, new PIDGains(0, 0, 0));
        pivotSetpoint = khome;
        
        pivotTimer = new Timer();
        pivotTimer.start();
        pivotTimer.reset(); 
        setLimits();
        updateMotionProfile();

        Timer.delay(0.2);
    }
public void setLimits(){
    //set soft limits and current limits for how far the manip can move
    pivotMotor.setSmartCurrentLimit(kCurrentLimit);
    
    // pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    // pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // // stops motor at 130 encoder clicks, (touching the ground)
    // pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 1.27);
    // // stops motor at 0 encoder clicks when reversing, (touching the robot)
    // pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) .07);
   }
   
   private void updateMotionProfile() {
    pivotTimer.reset();
   }


   
public double getRotations(){
  
    //gets position
    return pivotEncoder.getPosition();
}

public void armMoveVoltage(double volts) {
    //towerMotor.set(pid.calculate(getRotations(), setpoint) + feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
    // rotateMotor.setVoltage(volts+getAutoBalanceVolts());// sets voltage of arm -12 to 12 volts
    pivotMotor.setVoltage(volts);
    SmartDashboard.putNumber("Arm volts", volts);
}

public double getAutoBalanceVolts(){
    // Math.cos(theta) as more downward force increases near 0,180 degrees
   return (FF_kG*Math.cos(Math.toRadians(calculateDegreesRotation())));
}

public double calculateDegreesRotation(){
    double encoderClicksToDegrees = 8192.00/360.00;
    return (encoderClicksToDegrees*getRotations());
}
// public void runManual(double _power) {
//     //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
//     pivotSetpoint = pivotEncoder.getPosition();
//     targetState = new TrapezoidProfile.State(pivotSetpoint, 0.0);
//    // pivotTrapezoidProfile = new TrapezoidProfile(Constants.PIDConstants.kPivotConstraint, targetState, targetState);
//     //update the feedforward variable with the newly zero target velocity
//     //feedforward = Constants.Manuiplator.kArmFeedforward.calculate(pivotencoder.getPosition()+Constants.Manuiplator.kArmZeroCosineOffset, targetState.veEcity);
//     pivotMotor.set(_power + (feedforward / 12.0));
//     manualValue = _power;
//   }

public void runManual(double _power) {
   
     pivotMotor.set(_power);
     
  }

  public void runAutomatic(){
    double elapsedTime = pivotTimer.get();
    if(pivotTrapezoidProfile.isFinished(elapsedTime)){
      targetState = new TrapezoidProfile.State(pivotSetpoint, 0.0);
    }else{
        TrapezoidProfile.State state = new TrapezoidProfile.State(pivotEncoder.getPosition()
    , pivotEncoder.getVelocity());
        targetState = pivotTrapezoidProfile.calculate(elapsedTime, state, targetState);
    }
    feedforward = kArmFeedforward.calculate(pivotEncoder.getPosition(), targetState.velocity);
    pivotPID.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
  }

  public void setHome(){
    khome = pivotSetpoint;
  }
}