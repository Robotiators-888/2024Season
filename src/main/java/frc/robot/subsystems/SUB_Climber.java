package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import frc.robot.Constants.Climber;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Climber extends SubsystemBase {

//weight of robot : 115
//force of gravity: 511.19860099 Neutons
//511.19/2 NEO motors: 255.595 Neutons per neo disreguarding springs
//

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

  }



  public void runMotor(double pwr, int flag) {
    switch(flag){
        case 0:
        if(climberLeft.getEncoder().getPosition() > 10){
            climberLeft.setVoltage(pwr);
        }
        break;
        case 1:
        climberLeft.setVoltage(pwr);
        break;
    }
  }


}
