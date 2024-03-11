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
    climberRight = new CANSparkMax(41, MotorType.kBrushless);
    climberLeft = new CANSparkMax(40, MotorType.kBrushless);
    
    climberLeft.restoreFactoryDefaults();
    climberRight.restoreFactoryDefaults();

    climberLeft.setInverted(true);

    climberRight.setIdleMode(IdleMode.kBrake);
    climberLeft.setIdleMode(IdleMode.kBrake);
    //climberLeft.set

    climberLeft.burnFlash();
    climberRight.burnFlash();
  }

  /**
   * Runs the left climber motor at a specified power.
   * @param speed The speed to set the motor to as a decimal.
   */
  public void runLeft(double speed) {
   climberLeft.set(speed);
  }

   /**
   * Runs the left climber motor at a specified power.
   * @param speed The speed to set the motor to as a decimal.
   */
  public void runRight(double speed) {
   climberRight.set(speed);
  }


}
