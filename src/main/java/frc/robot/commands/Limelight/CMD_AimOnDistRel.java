// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;

public class CMD_AimOnDistRel extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  Pose2d tagPose;
  Integer targetId;

  Pose2d currentPose;
  Double positionError;

  private final PIDController robotAngleController = new PIDController( 1, 0, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_AimOnDistRel(SUB_Pivot pivot, SUB_Limelight limelight, SUB_Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    addRequirements(pivot, limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    var targetTransform = limelight.getTargetTransform();
    double xError = targetTransform.getZ();
    double yError = targetTransform.getX();
    
    currentPose = drivetrain.getPose(); // Robot's current pose
    positionError = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

    double angle = -Math.atan2(xError, yError); // x and y are not flipped???

    robotAngleController.setTolerance(0.07);
    robotAngleController.setSetpoint(angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("SPEAKER LOCK?", false);
    currentPose = drivetrain.getPose();
    pivot.goToAngle(pivot.distToPivotAngle.get(positionError) + 27);
    pivot.runAutomatic();

    SmartDashboard.putNumber("Distance error", positionError);
    drivetrain.drive(0, 0, robotAngleController.calculate(currentPose.getRotation().getRadians()),
     true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pivot.calculateDegreesRotation()-pivot.distToPivotAngle.get(positionError)) < 5 && robotAngleController.atSetpoint();
  }
}
