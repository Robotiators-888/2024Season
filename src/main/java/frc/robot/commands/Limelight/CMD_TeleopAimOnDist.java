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
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;

public class CMD_TeleopAimOnDist extends Command {
  SUB_Pivot pivot;
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  Pose2d tagPose;
  Integer targetId;

  Pose2d currentPose;
  Double positionError;

  double xError;
  double yError;
  double angle;

  CommandXboxController driverController;

  private final PIDController robotAngleController = new PIDController( 0.25, 0, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_TeleopAimOnDist(SUB_Pivot pivot, SUB_Limelight limelight, SUB_Drivetrain drivetrain, CommandXboxController driverController) {
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

    robotAngleController.setTolerance(0.04);
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();
    currentPose = drivetrain.getPose();
    positionError = Math.sqrt(Math.pow(tagPose.getX() - currentPose.getX(), 2)
                           + Math.pow(tagPose.getY() - currentPose.getY(), 2));

    xError = tagPose.getX() - currentPose.getX();
    yError = tagPose.getY() - currentPose.getY();
    angle = Math.atan2(yError, xError);
    
    // if (alliance.isPresent()) {
    //   if (alliance.get() == DriverStation.Alliance.Red){
    //     pivot.goToAngle((pivot.distToPivotAngle.get(positionError) + 27) - (Math.abs(currentPose.getRotation().getRadians() - 0) * 4));
    //   } else {
    //     pivot.goToAngle((pivot.distToPivotAngle.get(positionError) + 27)- (Math.abs(currentPose.getRotation().getRadians()) * 4));

    //   }
    // } 
    pivot.goToAngle((pivot.distToPivotAngle.get(positionError) + 27));
    // |currentpos - radians| * 5 - ()
    pivot.runAutomatic();

    SmartDashboard.putNumber("X Error", xError);
    SmartDashboard.putNumber("Y Error", yError);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Cur Rotation Radians", currentPose.getRotation().getRadians());
    SmartDashboard.putNumber("Distance error", positionError);

    // double ks = 0.1;
    // if (Math.abs(currentPose.getRotation().getRadians()-angle) <= 0.04){
    //   ks = 0;
    // } else if (robotAngleController.calculate(currentPose.getRotation().getRadians(), angle) < 0 ){
    //   ks *= -1;
    // }
    // ks=0;



      drivetrain.drive(
      -MathUtil.applyDeadband(Math.copySign(Math.pow(driverController.getRawAxis(1), 2), driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(Math.copySign(Math.pow(driverController.getRawAxis(0), 2), driverController.getRawAxis(0)), OIConstants.kDriveDeadband), 
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
    return Math.abs(pivot.calculateDegreesRotation()-pivot.distToPivotAngle.get(positionError)) < 5 
    && (Math.abs(currentPose.getRotation().getRadians()-angle) <= 0.04);
  }
}
