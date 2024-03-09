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
  }

  /**
   * Runs the left climber motor at a specified power.
   * @param speed The speed to set the motor to as a decimal.
   */
  public void runMotor(double speed) {
   climberLeft.set(speed);
  }
}
