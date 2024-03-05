// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.FieldRelativeAccel;
import frc.robot.utils.FieldRelativeSpeed;


/*
 * ASSUMES THAT THE SHOOTER IS ALREADY REVVED UP (SHOTS ARE INSTANT.)
 */
public class CMD_MovingAimingOnDist extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  SUB_Shooter shooter;
  
  Pose2d tagPose;
  Integer targetId;

  Pose2d currentPose;
  Double positionError;

  double xError;
  double yError;
  double angle;

  CommandXboxController controller;

  private final PIDController robotAngleController = new PIDController( 0.5, 0.01, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_MovingAimingOnDist(SUB_Pivot pivot, SUB_Limelight limelight, SUB_Drivetrain drivetrain, SUB_Shooter shooter, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(pivot, limelight, drivetrain);
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("SPEAKER LOCK?", false);

    currentPose = drivetrain.getPose(); // Robot's current pose
    double actualPositionError = Math.sqrt(Math.pow(tagPose.getX() - currentPose.getX(), 2)
                           + Math.pow(tagPose.getY() - currentPose.getY(), 2));

    xError = tagPose.getX() - currentPose.getX();
    yError = tagPose.getY() - currentPose.getY();

    FieldRelativeSpeed robotVel = drivetrain.getFieldRelativeSpeed();
    FieldRelativeAccel robotAccel = drivetrain.getFieldRelativeAccel(); 

    double shotTime = shooter.distToTimeMap.get(positionError);

    double virtualGoalX = tagPose.getX()
    - shotTime * (robotVel.vx + robotAccel.ax * 1);
    double virtualGoalY = tagPose.getY()
        - shotTime * (robotVel.vy + robotAccel.ay * 1);

    double virtualPositionError = Math.sqrt(Math.pow(virtualGoalX - currentPose.getX(), 2)
                           + Math.pow(virtualGoalY - currentPose.getY(), 2));

    angle = Math.atan2(virtualGoalY, virtualGoalX); // x and y are not flipped???

    currentPose = drivetrain.getPose();
    pivot.goToAngle(pivot.distToPivotAngle.get(virtualPositionError) + 27);
    pivot.runAutomatic();

    SmartDashboard.putNumber("X Error", xError);
    SmartDashboard.putNumber("Y Error", yError);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Cur Rotation Radians", currentPose.getRotation().getRadians());
    SmartDashboard.putNumber("Distance error", positionError);


    drivetrain.drive(
      -MathUtil.applyDeadband(Math.copySign(Math.pow(controller.getRawAxis(1), 2), controller.getRawAxis(1)), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(Math.copySign(Math.pow(controller.getRawAxis(0), 2), controller.getRawAxis(0)), OIConstants.kDriveDeadband),
      robotAngleController.calculate(currentPose.getRotation().getRadians(), angle),
     true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("SPEAKER LOCK?", true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pivot.calculateDegreesRotation()-pivot.distToPivotAngle.get(positionError)) < 5 && 
           Math.abs(currentPose.getRotation().getRadians()-angle) <= 0.07;
  }
}
