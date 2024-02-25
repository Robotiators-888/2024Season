// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;

public class CMD_AimOnDist extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  CommandXboxController controller;
  
  Pose2d tagPose;
  Integer targetId;

  Pose2d currentPose;
  Double positionError;

  private final PIDController robotAngleController = new PIDController( 1, 0, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_AimOnDist(SUB_Pivot pivot, SUB_Limelight limelight, SUB_Drivetrain drivetrain, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(pivot, limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    Alliance allianceColor;
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red){
        tagPose = drivetrain.at_field.getTagPose(4).get().toPose2d();
        targetId = 4;
      } else {
        tagPose = drivetrain.at_field.getTagPose(7).get().toPose2d(); 
        targetId = 7;
      }

    } else {
      end(true);
    }
    
    currentPose = drivetrain.getPose(); // Robot's current pose
    positionError = Math.sqrt(Math.pow(tagPose.getX() - currentPose.getX(), 2)
                           + Math.pow(tagPose.getY() - currentPose.getY(), 2));

    double xError = tagPose.getX() - currentPose.getX();
    double yError = tagPose.getY() - currentPose.getY();
    double angle = Math.atan2(yError, xError); // x and y are not flipped???

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
    drivetrain.drive(-Math.copySign(Math.pow(controller.getRawAxis(1), 2), controller.getRawAxis(1)),
       -Math.copySign(Math.pow(controller.getRawAxis(0), 2), controller.getRawAxis(0)), 
       robotAngleController.calculate(currentPose.getRotation().getRadians()),
     false, true);
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
