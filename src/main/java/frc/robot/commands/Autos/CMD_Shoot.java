// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shoot extends Command {
  SUB_Shooter shooter;
  SUB_Index index;
  

  /** Creates a new CMD_Shoot. */
  public CMD_Shoot(SUB_Shooter shooter, SUB_Index index) {
    this.shooter = shooter;
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shootFlywheelOnRPM(4000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("cmd END", false);
    //shooter.shootFlywheelOnRPM(4000);
    if(shooter.getFlywheelRPM() >= 3500){
      index.setMotorSpeed(0.75);
      Timer.delay(0.5);
      
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("cmd END", true);
    shooter.setMotorSpeed(0);
    index.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}