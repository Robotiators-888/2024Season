package frc.robot.commands;

import frc.robot.subsystems.SUB_Drivetrain;
import java.util.Optional;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CMD_AbsoluteDriveToTarget extends Command {
  SUB_Drivetrain drivetrain;

  private final PIDController xController = new PIDController(0.25, 0, 0); // 3, 0, 0
  private final PIDController yController = new PIDController(0.25, 0, 0); // 3/5, 0, 0
  private final PIDController omegaController = new PIDController( 1, 0, 0); // 0.25, 0, 0
  private final Pose3d goalPose;
  
  /**
   * Drive to goal using limelight
   * @param drivetrain The drivetrain subsystem
   * @param goalPose The goal position
   */
  public CMD_AbsoluteDriveToTarget(SUB_Drivetrain drivetrain, Optional<Pose3d> goalPose) {
    this.drivetrain = drivetrain;
    this.goalPose = goalPose.get();

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(0.07);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var robotPose = drivetrain.getPose();
    SmartDashboard.putNumber("Goal Pose Y", goalPose.getY());
    SmartDashboard.putNumber("Robot Pose Y", robotPose.getY());
    SmartDashboard.putNumber("Goal Pose Rotation", goalPose.getRotation().getAngle());
    SmartDashboard.putNumber("Robot Pose Rotation", robotPose.getRotation().getRadians());

    var xSpeed = xController.calculate(robotPose.getX(), goalPose.getX());
    var ySpeed = yController.calculate(robotPose.getY(), goalPose.getY());
    // In this case, it has to be what angle/rotation we want the robot to be in.
    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians(), goalPose.getRotation().getAngle());

    drivetrain.drive(xSpeed, ySpeed, omegaSpeed, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    // drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint());
  }
}