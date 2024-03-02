// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shoot extends Command {
  SUB_Shooter shooter;
  SUB_Index index;
  boolean flag;
  

  /** Creates a new CMD_Shoot. */
  public CMD_Shoot(SUB_Shooter shooter, SUB_Index index, SUB_Drivetrain drivetrain, SUB_Intake intake, SUB_Pivot pivot) {
    this.shooter = shooter;
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, index, drivetrain, intake, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    shooter.shootFlywheelOnRPM(4000);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("cmd END", false);
    if(!(shooter.getFlywheelRPM() >= 3500)){
      shooter.shootFlywheelOnRPM(4000);
    }else{
      index.setMotorSpeed(.5);
      flag = true;
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(0.5);
    SmartDashboard.putBoolean("cmd END", true);
    shooter.setMotorSpeed(0);
    index.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flag;
  }
}
