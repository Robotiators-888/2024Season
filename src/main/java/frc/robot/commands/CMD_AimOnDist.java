// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;

public class CMD_AimOnDist extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  Pose2d tagPose;
  Integer targetId;

  Pose2d currentPose;
  Double positionError;

  //private final PIDController robotAngleController = new PIDController( 1, 0, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_AimOnDist(SUB_Pivot pivot, SUB_Limelight limelight, SUB_Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    addRequirements(pivot, limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(DriverStation.getAlliance().get()){
      case Blue: 
        tagPose = drivetrain.at_field.getTagPose(4).get().toPose2d(); 
        targetId = 4;
        break;
      case Red: 
        tagPose = drivetrain.at_field.getTagPose(7).get().toPose2d();
        targetId = 7;
        break;
    }
    
    currentPose = drivetrain.getPose(); // Robot's current pose
    positionError = Math.sqrt(Math.pow(tagPose.getX() - currentPose.getX(), 2)
                           + Math.pow(tagPose.getY() - currentPose.getY(), 2));

    double angle = currentPose.getRotation().getRadians();
    double xError = tagPose.getX() - currentPose.getX();
    double yError = tagPose.getY() - currentPose.getY();

    if (xError > 0){
      if (yError < 0){
        angle = Math.atan(yError/xError);
      } else {
        angle = -Math.atan(yError/xError);
      }
    } else {
      if (yError < 0){
        angle = Math.PI - Math.atan(yError/xError);
      } else {
        angle = -Math.PI - Math.atan(yError/xError);
      }
    }
    //robotAngleController.setTolerance(0.07);
    //robotAngleController.setSetpoint(angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("SPEAKER LOCK?", false);
    currentPose = drivetrain.getPose();
    pivot.goToAngle(pivot.distToPivotAngle.get(positionError));
    pivot.runAutomatic();
    SmartDashboard.putNumber("Distance error", positionError);
    // drivetrain.drive(0, 0, robotAngleController.calculate(currentPose.getRotation().getRadians()),
    //  false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pivot.calculateDegreesRotation()-pivot.distToPivotAngle.get(positionError)) < 5;
  }
}
