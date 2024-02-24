package frc.robot.subsystems;

import static frc.robot.Constants.Pivot.*;

import java.util.function.Supplier;

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
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Pivot extends SubsystemBase {
    public InterpolatingDoubleTreeMap constantApplicationMap = new InterpolatingDoubleTreeMap();
    private final CANSparkMax pivotMotor;
    public final SparkAbsoluteEncoder rotateEncoder;
    private final RelativeEncoder rotateRelativeEncoder;
    private Timer pivotTimer;
    private TrapezoidProfile pivotTrapezoidProfile;
    private SparkPIDController pivotPID;
    private double pivotSetpoint;
    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    public InterpolatingDoubleTreeMap distToPivotAngle = new InterpolatingDoubleTreeMap();
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;


 public SUB_Pivot(){
        pivotMotor = new CANSparkMax(kPIVOT_ROTATE_MOTOR_CANID, MotorType.kBrushless);
        rotateEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        rotateRelativeEncoder = pivotMotor.getEncoder();
        pivotTrapezoidProfile = new TrapezoidProfile(kArmMotionConstraint);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        rotateEncoder.setPositionConversionFactor(360);
        rotateEncoder.setInverted(true);
        rotateEncoder.setZeroOffset(0); // We are accepting that this is broken
        pivotMotor.setSmartCurrentLimit(50);
        // pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // pivotEncoder.setVelocityConversionFactor(1.0/4.0 * 2 * Math.PI);
        // pivotEncoder.setPositionConversionFactor(1.0/4.0 * 2 * Math.PI);
        // rotateRelativeEncoder.setPositionConversionFactor(1.0/(300.0)*2*Math.PI);
        // rotateRelativeEncoder.setPosition(pivotEncoder.getPosition());
        pivotMotor.burnFlash();
        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(rotateEncoder);
        PIDGains.setSparkMaxGains(pivotPID, new PIDGains(0, 0, 0));
        pivotSetpoint = khome;
        
        pivotTimer = new Timer();
        pivotTimer.start();
        pivotTimer.reset(); 
        setLimits();
        updateMotionProfile();

        constantApplicationMap.put(107.0 , 0.04);
        constantApplicationMap.put(95.0, 0.09);
        constantApplicationMap.put(61.0, 0.04);

        distToPivotAngle.put(Units.feetToMeters(3), 59.0);
        distToPivotAngle.put(Units.feetToMeters(6), 45.0);
        distToPivotAngle.put(Units.feetToMeters(9), 36.0);
        //Timer.delay(0.2);
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
   
   public void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(rotateEncoder.getPosition(), rotateEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(pivotSetpoint, 0.0);
    pivotTimer.reset();
   }


   
public double getRotations(){
    return rotateEncoder.getPosition();
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
    return getRotations() - 27;
}

public void goToAngle(double angle){
  if((angle<Constants.Pivot.kMaxArmAngle) && (angle>Constants.Pivot.kMinArmAngle)){
    pivotSetpoint = angle+27;
  }
}

public void runManual(double _power) {
     pivotMotor.set(_power);
}

  public void runAutomatic(){
    double elapsedTime = pivotTimer.get();
    if(pivotTrapezoidProfile.isFinished(elapsedTime)){
      targetState = new TrapezoidProfile.State(pivotSetpoint, 0.0);
    }else{
        TrapezoidProfile.State state = new TrapezoidProfile.State(rotateEncoder.getPosition()
    , rotateEncoder.getVelocity());
        targetState = pivotTrapezoidProfile.calculate(.02, state, targetState);
    }
    feedforward = kArmFeedforward.calculate(targetState.velocity) + 
                  12 * constantApplicationMap.get(rotateEncoder.getPosition());
    pivotPID.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
  }

  public void setHome(){
    khome = pivotSetpoint;
  }

  public double calculateConstantApp(Supplier<Double> encoderPosition){
    return constantApplicationMap.get(encoderPosition.get());
  }

  public void periodic(){
    SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Pivot Rotations", getRotations());
    SmartDashboard.putNumber("Pivot FF", feedforward);
    SmartDashboard.putNumber("Pivot % out", pivotMotor.getAppliedOutput());
  }
}


/*
 * 10.5ft 55 Degree
 * Amp 500RPM 58 ~ 60
 * 
 */
