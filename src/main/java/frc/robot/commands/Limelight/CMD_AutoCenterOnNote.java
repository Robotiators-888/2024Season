// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.Vision.SUB_Limelight;
import frc.robot.subsystems.Vision.SUB_PhotonVision;

import static frc.robot.Constants.Pivot.*;

import org.photonvision.targeting.PhotonTrackedTarget;
public class CMD_AutoCenterOnNote extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  SUB_PhotonVision photonVision;


  CommandXboxController driverController;

  private final PIDController robotAngleController = new PIDController( 0.01, 0, .0005); // 0.25, 0, 0
  private PhotonTrackedTarget note;
  
  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_AutoCenterOnNote(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    note = photonVision.getBestNote();
    robotAngleController.setTolerance(4.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    note = photonVision.getBestNote();

    if (note == null){
      
    } else {
      drivetrain.drive(
      0,
      -MathUtil.clamp(robotAngleController.calculate(note.getYaw(), 0), -0.3, 0.3),
      0,
      false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !SUB_PhotonVision.getInstance().hasResults;
  }
}
