// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.Vision.*;

public class CMD_AutoAimOnDist_NO_DRIVE extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  Pose2d tagPose;
  Integer targetId;
  Double positionError;

  double xError;
  double yError;
  double angle;


  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_AutoAimOnDist_NO_DRIVE(SUB_Pivot pivot, SUB_Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.  
    this.pivot = pivot;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red){
        tagPose = drivetrain.at_field.getTagPose(4).get().toPose2d();
        targetId = 4;
      } else {
        tagPose = drivetrain.at_field.getTagPose(7).get().toPose2d(); 
        targetId = 7;
      }
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
    }

    SmartDashboard.putBoolean("SPEAKER LOCK?", false);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    positionError = Math.sqrt(Math.pow(tagPose.getX() - currentPose.getX(), 2)
                           + Math.pow(tagPose.getY() - currentPose.getY(), 2));

    xError = tagPose.getX() - currentPose.getX();
    yError = tagPose.getY() - currentPose.getY();
    angle = Math.atan2(yError, xError);
    
    pivot.goToAngle((pivot.distToPivotAngle.get(positionError) + 27));
    pivot.runAutomatic();

    SmartDashboard.putNumber("X Error", xError);
    SmartDashboard.putNumber("Y Error", yError);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Cur Rotation Radians", MathUtil.angleModulus(currentPose.getRotation().getRadians()));
    SmartDashboard.putNumber("Distance error", positionError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("SPEAKER LOCK?", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pivot.calculateDegreesRotation()-pivot.distToPivotAngle.get(positionError)) < 5;
  }
}
