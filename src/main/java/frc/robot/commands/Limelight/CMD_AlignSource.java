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
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;
import static frc.robot.Constants.Pivot.*;
public class CMD_AlignSource extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  Pose3d tagPose;
  Integer targetId;

  CommandXboxController driverController;

  private final PIDController robotAngleController = new PIDController( 0.5, 0.01, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_AlignSource(SUB_Pivot pivot, SUB_Limelight limelight, SUB_Drivetrain drivetrain, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    addRequirements(pivot, limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red){
        tagPose = drivetrain.at_field.getTagPose(9).get();
        targetId = 9;
      } else {
        tagPose = drivetrain.at_field.getTagPose(2).get();
        targetId = 2;
      }
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
    }

    robotAngleController.setTolerance(0.07);
    SmartDashboard.putNumber("rot radians", tagPose.getRotation().getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    pivot.goToAngle(kSourceAngle);
    pivot.runAutomatic();
    double target = tagPose.getRotation().getAngle() + Math.PI;

    if (target > Math.PI) {
      target = -Math.PI + tagPose.getRotation().getAngle();
    }

    drivetrain.drive(
      -MathUtil.applyDeadband(Math.copySign(Math.pow(driverController.getRawAxis(1), 2), driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(Math.copySign(Math.pow(driverController.getRawAxis(0), 2), driverController.getRawAxis(0)), OIConstants.kDriveDeadband),
      
      robotAngleController.calculate(currentPose.getRotation().getRadians(), target),
     false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pivot.calculateDegreesRotation()-kSourceAngle) < 5 && 
    robotAngleController.atSetpoint();
  }
}
