// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.CMD_AbsoluteDriveToTarget;
import frc.robot.commands.CMD_DriveToTarget;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.utils.LogiUtils;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  public static SUB_Limelight limelight = new SUB_Limelight();

  Joystick joystick = new Joystick(0);
  LogiUtils DriverC = new LogiUtils(0);
  LogiUtils logiUtils1 = new LogiUtils(1);
  JoystickButton leftBumperC = DriverC.getLeftBumperButtonPressed();
  JoystickButton RightBumperC = DriverC.getRightBumperButtonPressed();
  JoystickButton leftBumper = logiUtils1.getLeftBumperButtonPressed();
  JoystickButton rightBumper = logiUtils1.getRightBumperButtonPressed();
  JoystickButton aButton = logiUtils1.getAButtonPressed(); // Ground
  JoystickButton yButton = logiUtils1.getYButtonPressed(); // Stow/up
  JoystickButton xButton = logiUtils1.getXButtonPressed(); // Single Feed
  JoystickButton bButton = logiUtils1.getBButtonPressed(); // Scoring Height
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
    
    // log = DataLogManager.getLog();
    // poseEntry = new DoubleArrayLogEntry(log, "odometry/pose");
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
    Optional<Pose3d> p3d = Optional.of(new Pose3d(new Pose2d(1, 1, Rotation2d.fromDegrees(45))));
    startButton.whileTrue(new CMD_DriveToTarget(limelight, drivetrain)).onFalse(new InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
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

  public void periodic(){
    Pose2d visionPose = limelight.getPose();
    if (visionPose != null){
      drivetrain.addVisionMeasurement(visionPose);
    }
  }
}
