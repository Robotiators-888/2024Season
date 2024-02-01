// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.utils.AutoGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
   public static SUB_Shooter shooter = new SUB_Shooter();
   public static SUB_Index index = new SUB_Index();
   public static SUB_Intake intake = new SUB_Intake();
   public static SUB_Pivot pivot = new SUB_Pivot();
   public static AutoGenerator autos = new AutoGenerator(drivetrain);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController DriverC =
      new CommandXboxController(OIConstants.kDriverControllerPort);

    private final CommandXboxController OperatorC = 
    new CommandXboxController(OIConstants.kDriver2ControllerPort);   
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
    
    shooter.setDefaultCommand(new RunCommand(()->shooter.setMotorSpeed(0), shooter));
    index.setDefaultCommand(new RunCommand(()->index.setMotorSpeed(0), index));
    
    DriverC.a().whileTrue(new InstantCommand(()->index.setMotorSpeed(.5)));
    DriverC.b().whileTrue((new RunCommand(()->shooter.setMotorSpeed(-0.5), shooter)));

    
    // log = ataLogManager.getLog();
    // poseEntry = new DoubleArrayLogEntry(log, "odometry/pose");
    intake.setDefaultCommand(new RunCommand(()->intake.setMotorSpeed(0), intake));
    DriverC.x().whileTrue((new InstantCommand(()->intake.setMotorSpeed(Constants.Intake.kIntakeSpeed))));
    DriverC.y().whileTrue(new InstantCommand(()->intake.setMotorSpeed(Constants.Intake.kIndexingSpeed)));
    DriverC.leftBumper().whileTrue(new InstantCommand(()->intake.setMotorSpeed(Constants.Intake.kOutakeSpeed)));
    // new Trigger(() -> 
    //   Math.abs(Math.pow(DriverC.getRawAxis(3), 2) - Math.pow(DriverC.getRawAxis(2), 3)) > Constants.Pivot.kPivotManualDeadband
    //   ).whileTrue(new RunCommand(
    //     () ->
    //     pivot.runManual((Math.pow(DriverC.getRawAxis(3), 2) - Math.pow(DriverC.getRawAxis(2), 3)) * Constants.Pivot.kArmManualScale)
    //     , pivot));

    pivot.setDefaultCommand(new RunCommand(()->pivot.runManual(0), pivot));

    DriverC.povUp().whileTrue(new RunCommand(()->pivot.runManual(.2), pivot));    DriverC.povUp().whileTrue(new RunCommand(()->pivot.runManual(.2)));
    DriverC.povDown().whileTrue(new RunCommand(()->pivot.runManual(-.2), pivot));


    //OperatorC.a().onTrue(new InstantCommand(()-> pivot.setHome()));
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autos.getSelectedAuto();
  }
}
