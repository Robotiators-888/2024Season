// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;

public class CMD_Intake extends Command {
  private SUB_Intake intake;
  private SUB_Index index;

  /** Creates a new CMD_Intake. */
  public CMD_Intake(SUB_Intake intake, SUB_Index index) {
    this.intake = intake;
    this.index = index;
    addRequirements(intake, index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.setMotorSpeed(Constants.Intake.kIndexSpeed);
    intake.setMotorSpeed(Constants.Intake.kIntakingSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(index.getTopBannerSensor()){
      index.setMotorSpeed(0.1);
      Timer.delay(0.1);
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setMotorSpeed(0);
    intake.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
