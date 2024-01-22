// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.CMD_AbsoluteDriveToTarget;
import frc.robot.commands.CMD_RelativeDriveToTarget;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.utils.LogiUtils;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static SUB_Limelight limelight = new SUB_Limelight();
  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();

  Joystick joystick = new Joystick(0);
  LogiUtils DriverC = new LogiUtils(0);
  LogiUtils logiUtils1 = new LogiUtils(1);
  JoystickButton leftBumperC = DriverC.getLeftBumperButtonPressed();
  JoystickButton RightBumperC = DriverC.getRightBumperButtonPressed();
  JoystickButton leftBumper = logiUtils1.getLeftBumperButtonPressed();
  JoystickButton rightBumper = logiUtils1.getRightBumperButtonPressed();
  JoystickButton aButton = logiUtils1.getAButtonPressed();
  JoystickButton yButton = logiUtils1.getYButtonPressed(); 
  JoystickButton xButton = logiUtils1.getXButtonPressed(); 
  JoystickButton bButton = logiUtils1.getBButtonPressed(); 
  JoystickButton startButton = logiUtils1.getStartButtonPressed();
  JoystickButton backButton = logiUtils1.getBackButtonPressed();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(Math.pow(DriverC.getRawAxis(1), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(DriverC.getRawAxis(0), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriverC.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                drivetrain));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Pose3d op3d = drivetrain.at_field.getTagPose(4).get();
    Pose3d op = new Pose3d(new Pose2d(op3d.getX()-1, op3d.getY(), Rotation2d.fromDegrees(0)));
    Optional<Pose3d> p3d = Optional.of(new Pose3d(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(45))));
    startButton.whileTrue(new CMD_RelativeDriveToTarget(limelight, drivetrain)).onFalse(new InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
    backButton.whileTrue(new CMD_AbsoluteDriveToTarget(drivetrain, p3d)).onFalse(new InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void robotPeriodic(){
    Pose2d visionPose = limelight.getPose();
    if (!visionPose.equals(new Pose2d())){
      // Check if vision pose is within one meter of the current estiamted pose 
      // to avoid abnormalities with vision (detecting a tag that isn't present) from
      // affecting the accuracy of our pose measurement.
      Transform2d t2d = visionPose.minus(drivetrain.getPose());
      double dist = Math.sqrt(Math.pow(t2d.getX(), 2) + Math.pow(t2d.getY(), 2));
      if (dist <= 1){
        drivetrain.addVisionMeasurement(visionPose, limelight.getCaptureLatency() + limelight.getPipelineLatency());
      }
    }
  }
}
