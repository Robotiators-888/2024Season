package frc.robot.subsystems;

import static frc.robot.Constants.Pivot.*;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.libs.PIDGains;
import frc.robot.Constants.Pivot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Pivot extends SubsystemBase {
  public static SUB_Pivot INSTANCE = null;

  public InterpolatingDoubleTreeMap constantApplicationMap = new InterpolatingDoubleTreeMap();
  private final CANSparkMax pivotMotor;
  public final SparkAbsoluteEncoder rotateEncoder;
  private final RelativeEncoder rotateRelativeEncoder;
  private Timer pivotTimer;
  private TrapezoidProfile pivotTrapezoidProfile;
  private SparkPIDController pivotPID;
  private double pivotSetpoint;
  private TrapezoidProfile.State targetState;
  private TrapezoidProfile.State currentState;

  private TrapezoidProfile.State nextState;
  private double feedforward;
  private double manualValue;
  // Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
  public InterpolatingDoubleTreeMap distToPivotAngle = new InterpolatingDoubleTreeMap();
  double gravitional_force_in_Kg = (lbsArm * 4.44822162) / 9.8;

  public static SUB_Pivot getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Pivot();
    }

    return INSTANCE;
  }

  private SUB_Pivot() {
    pivotMotor = new CANSparkMax(kPIVOT_ROTATE_MOTOR_CANID, MotorType.kBrushless);
    rotateEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rotateRelativeEncoder = pivotMotor.getEncoder();
    pivotTrapezoidProfile = new TrapezoidProfile(kArmMotionConstraint);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
    pivotMotor.setInverted(false);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    rotateEncoder.setPositionConversionFactor(360);
    rotateEncoder.setVelocityConversionFactor(360);
    rotateEncoder.setInverted(true);
    rotateEncoder.setZeroOffset(Pivot.kPivotOffset); // We are accepting that this is broken
    pivotMotor.setSmartCurrentLimit(50);
    // pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // pivotEncoder.setVelocityConversionFactor(1.0/4.0 * 2 * Math.PI);
    // pivotEncoder.setPositionConversionFactor(1.0/4.0 * 2 * Math.PI);
    // rotateRelativeEncoder.setPositionConversionFactor(1.0/(300.0)*2*Math.PI);
    // rotateRelativeEncoder.setPosition(pivotEncoder.getPosition());
    pivotPID = pivotMotor.getPIDController();
    pivotPID.setFeedbackDevice(rotateEncoder);
    PIDGains.setSparkMaxGains(pivotPID, new PIDGains(0, 0, 0));
    pivotPID.setOutputRange(-0.2, 0.2);
    setPIDF(pivotPID, 0.020, 0, 0.00, 0);

    Timer.delay(.1);

    System.out.println("DONE");
    pivotMotor.burnFlash();

    pivotSetpoint = khome;

    pivotTimer = new Timer();
    pivotTimer.start();
    pivotTimer.reset();
    setLimits();
    targetState = new TrapezoidProfile.State(75, 0.0);
    currentState = new TrapezoidProfile.State(75, 0.0);

    constantApplicationMap.put(107.0, -0.04);
    constantApplicationMap.put(95.0, -0.08);
    constantApplicationMap.put(61.0, -0.03);

    distToPivotAngle.put(Units.inchesToMeters(36.0), 86.0 - 27.0);
    distToPivotAngle.put(Units.inchesToMeters(49), 59.0);
    distToPivotAngle.put(Units.inchesToMeters(61), 53.0);
    distToPivotAngle.put(Units.inchesToMeters(73), 45.0);
    distToPivotAngle.put(Units.inchesToMeters(87), 44.0);
    distToPivotAngle.put(Units.inchesToMeters(85), 41.0);

    distToPivotAngle.put(Units.inchesToMeters(97), 37.0);
    distToPivotAngle.put(Units.inchesToMeters(109), 36.0);
    distToPivotAngle.put(Units.inchesToMeters(121), 34.0);
    // distToPivotAngle.put()

    // Extrapolated from lin reg on previous ^ data
    distToPivotAngle.put(Units.inchesToMeters(152.283), 29.611);
    distToPivotAngle.put(Units.inchesToMeters(164.016), 26.7);
    // distToPivotAngle.put(Units.inchesToMeters(13*12 +13), -4.61*13 +72.57);
    // distToPivotAngle.put(Units.inchesToMeters(14*12 +13), -4.61*14 +72.57);
    // distToPivotAngle.put(Units.inchesToMeters(15*12 +13), -4.61*15 +72.57);
    // distToPivotAngle.put(Units.inchesToMeters(16*12 +13), -4.61*16 +72.57);

    // Timer.delay(0.2);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    pivotSetpoint = rotateEncoder.getPosition();
  }

  public double getAngle(){
    return rotateEncoder.getPosition();
  }

  public void setLimits() {
    // set soft limits and current limits for how far the manip can move
    pivotMotor.setSmartCurrentLimit(kCurrentLimit);

    // pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    // pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // // stops motor at 130 encoder clicks, (touching the ground)
    // pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 1.27);
    // // stops motor at 0 encoder clicks when reversing, (touching the robot)
    // pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) .07);
  }

  public void setPivotSetpoint(double newSetpoint) {
    pivotSetpoint = newSetpoint;
  }

  public void setPIDF(SparkPIDController pid, double P, double I, double D, double F) {
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }

  public void setSetpointToPivot() {
    pivotSetpoint = rotateEncoder.getPosition();
  }

  public double getRotations() {
    return rotateEncoder.getPosition();
  }

  public void armMoveVoltage(double volts) {
    // towerMotor.set(pid.calculate(getRotations(), setpoint) +
    // feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
    // rotateMotor.setVoltage(volts+getAutoBalanceVolts());// sets voltage of arm
    // -12 to 12 volts
    pivotMotor.setVoltage(volts);
    SmartDashboard.putNumber("Arm volts", volts);
  }

  public double getAutoBalanceVolts() {
    // Math.cos(theta) as more downward force increases near 0,180 degrees
    return (FF_kG * Math.cos(Math.toRadians(calculateDegreesRotation())));
  }

  public double calculateDegreesRotation() {
    return getRotations() - 27;
  }

  public void goToAngle(double angle) {
    pivotSetpoint = Math.min(kMaxArmAngle, Math.max(kMinArmAngle, angle));
    targetState = new TrapezoidProfile.State(pivotSetpoint, 0.0);
    currentState = new TrapezoidProfile.State(rotateEncoder.getPosition(), rotateEncoder.getVelocity());
  }

  public void runManual(double _power) {
    pivotMotor.set(_power);
    setPivotSetpoint(rotateEncoder.getPosition());
  }

  public void runAutomatic() {
    // currentState = pivotTrapezoidProfile.calculate(.02, currentState,
    // targetState);
    feedforward = 12 * constantApplicationMap.get(rotateEncoder.getPosition());
    pivotPID.setReference(pivotSetpoint, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
    // pivotMotor.setVoltage(feedforward);
  }

  public void setHome() {
    khome = pivotSetpoint;
  }

  public double calculateConstantApp(Supplier<Double> encoderPosition) {
    return constantApplicationMap.get(encoderPosition.get());
  }

  public void periodic() {
    SmartDashboard.putNumber("Pivot/Pivot Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Pivot/Pivot Rotations", getRotations());
    SmartDashboard.putNumber("Pivot/Current Velocity", rotateEncoder.getVelocity());
    SmartDashboard.putNumber("Pivot/TargetVelocity", currentState.velocity);
    SmartDashboard.putNumber("Pivot/Next position", currentState.position);
    SmartDashboard.putNumber("Pivot/Pivot FF", feedforward);
    SmartDashboard.putNumber("Pivot/Pivot % out", pivotMotor.getAppliedOutput());

  }
}

/*
 * 10.5ft 55 Degree
 * Amp 500RPM 58 ~ 60
 * 
 */
