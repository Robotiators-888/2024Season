// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_ShootSEQ extends Command {
  /** Creates a new CMD_ShootSEQ. */
  SUB_Shooter shooter;
  SUB_Index index;
  SUB_Pivot pivot;
  

  /** Creates a new CMD_Shoot. */
  public CMD_ShootSEQ(SUB_Shooter shooter, SUB_Index index, SUB_Pivot pivot) {
    this.shooter = shooter;
    this.index = index;
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, index, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setPivotSetpoint(Constants.Pivot.kAmpAngleSP);
    new CMD_Shoot(shooter, index).schedule();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
