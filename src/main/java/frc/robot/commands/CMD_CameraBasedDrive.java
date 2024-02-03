// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import static frc.robot.Constants.Limelight.*;

public class CMD_CameraBasedDrive extends Command {

  private final PIDController xController = new PIDController(0.25, 0, 0); // 3, 0, 0
  private final PIDController yController = new PIDController(0.25, 0, 0); // 3/5, 0, 0
  private final PIDController omegaController = new PIDController( 1, 0, 0); // 0.25, 0, 0

  SUB_Drivetrain drivetrain;
  SUB_Limelight limelight;
  /** Creates a new CMD_CameraBasedDrive. */
  public CMD_CameraBasedDrive(SUB_Drivetrain drivetrain, SUB_Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double d_numer = HEIGHT_TARGET_METERS - HEIGHT_CAMERA_METERS;
    double d_denom = Math.tan(CAMERA_ANGLE_DEGREES + limelight.getTargetPitch());
    double xError = d_numer/d_denom; // In inches
    double angleError = limelight.getTx();

    

    SmartDashboard.putNumber("Distance", xError);
    drivetrain.drive(xController.calculate(xError), 0, omegaController.calculate(angleError), false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false, true);
  }

  // Returns true when the command should end.6
  @Override
  public boolean isFinished() {
    return !limelight.getTv() || (xController.atSetpoint() && omegaController.atSetpoint()); // Is at setpoint
  }
}
