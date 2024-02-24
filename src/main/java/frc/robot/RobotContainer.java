// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.CMD_AbsoluteDriveToTarget;
import frc.robot.commands.CMD_AimOnDist;
import frc.robot.commands.CMD_RelativeDriveToTarget;
import frc.robot.commands.SEQ_StaticAimedShot;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.utils.AutoGenerator;
import frc.robot.subsystems.SUB_Limelight;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
   public static SUB_Limelight limelight = new SUB_Limelight();
   public static AutoGenerator autos = new AutoGenerator(drivetrain, index, intake, shooter, pivot, limelight);

  


  CommandXboxController DriverC = new CommandXboxController(OIConstants.kDriverControllerPort);

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

    
    shooter.setDefaultCommand(new RunCommand(()->shooter.shootFlywheelOnRPM(0), shooter));
    index.setDefaultCommand(new RunCommand(()->index.setMotorSpeed(0), index));
    intake.setDefaultCommand(new RunCommand(()->intake.setMotorSpeed(0), intake));
    pivot.setDefaultCommand(new RunCommand(()->pivot.runManual(pivot.calculateConstantApp(()->pivot.getRotations())), pivot));

    //new Trigger(()->(index.bannersensor())).whileTrue(new RunCommand(()->index.setMotorSpeed(0)));
    
     /* ================== *\
            Driver One 
     \* ================== */
     
    OperatorC.leftBumper().onTrue(new InstantCommand(()->
      SUB_Shooter.MANUAL_RPM -= 250
    )); // Decrease manual RPM by 100

    OperatorC.rightBumper().onTrue(
      new InstantCommand(()->
        SUB_Shooter.MANUAL_RPM += 250
    )); // Increase manual RPM by 100
    //pivot.setDefaultCommand(new RunCommand(()->pivot.runManual(0.05), pivot));

    //new Trigger(()->(index.bannersensor())).whileTrue(new RunCommand(()->index.setMotorSpeed(0)));
    
     /* ================== *\
            Driver One 
     \* ================== */ 

    // log = ataLogManager.getLog();
    // poseEntry = new DoubleArrayLogEntry(log, "odometry/pose");
    
   // DriverC.x().whileTrue((new RunCommand(()->intake.setMotorSpeed(Constants.Intake.kIntakeSpeed), intake)));
    // new Trigger(() -> 
    //   Math.abs(Math.pow(DriverC.getRawAxis(3), 2) - Math.pow(DriverC.getRawAxis(2), 3)) > Constants.Pivot.kPivotManualDeadband
    //   ).whileTrue(new RunCommand(
    //     () ->
    //     pivot.runManual((Math.pow(DriverC.getRawAxis(3), 2) - Math.pow(DriverC.getRawAxis(2), 3)) * Constants.Pivot.kArmManualScale)
    //     , pivot));

    OperatorC.b().whileTrue(new CMD_AimOnDist(pivot, limelight, drivetrain).andThen(
      new InstantCommand(()->SmartDashboard.putBoolean("SPEAKER LOCK?", true))));

    OperatorC.a().whileTrue(new SequentialCommandGroup(
      new InstantCommand(()->pivot.goToAngle(59.0)),
      new InstantCommand(()->pivot.updateMotionProfile()),
      new RunCommand(()->pivot.runAutomatic())
    ));
    
    DriverC.b().whileTrue(new RunCommand(()->shooter.shootFlywheelOnRPM(SUB_Shooter.MANUAL_RPM))).onFalse(new InstantCommand(()->shooter.shootFlywheelOnRPM(0)));
     
    DriverC.a().whileTrue(
    new ParallelCommandGroup(
      new RunCommand(()->index.setMotorSpeed(.5), index),
      new RunCommand(()->intake.setMotorSpeed(Constants.Intake.kIndexingSpeed))).until(
        ()->index.getTopBannerSensor()
    )).onFalse(new ParallelCommandGroup(

      new InstantCommand(()->index.setMotorSpeed(0)),
      new InstantCommand(()->intake.setMotorSpeed(0))
    )); // Suspicious if it will work or not, if it doesn't, just put onTrue();

    DriverC.x().whileTrue((new RunCommand(()->index.setMotorSpeed(-0.25), index))); //Drive Index OUT

    DriverC.b().whileTrue(
        new ParallelCommandGroup(
          new RunCommand(()->shooter.shootFlywheelOnRPM(4000), shooter),
          new SequentialCommandGroup(
            new WaitUntilCommand(()->shooter.getFlywheelRPM() == 4000),
            new RunCommand(()->index.setMotorSpeed(0.5), index)
          )
        )
    ); // Spin Shooter OUT
    DriverC.rightBumper().whileTrue(new RunCommand(()->shooter.setMotorSpeed(-0.25), shooter)); // Spin Shooter IN
    
    DriverC.y().whileTrue(new RunCommand(()->intake.setMotorSpeed(-Constants.Intake.kIndexingSpeed), intake)); // Drive Intake IN
    DriverC.leftBumper().whileTrue(new RunCommand(()->intake.setMotorSpeed(-Constants.Intake.kOutakeSpeed), intake)); //Drive Intake OUT
    DriverC.povUp().whileTrue(new RunCommand(()->pivot.runManual(-0.2), pivot));    
    DriverC.povDown().whileTrue(new RunCommand(()->pivot.runManual(0.2), pivot));

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
    // Pose3d op3d = drivetrain.at_field.getTagPose(4).get();
    // Pose3d op = new Pose3d(new Pose2d(op3d.getX()-3, op3d.getY()-1, Rotation2d.fromDegrees(0)));
    // Optional<Pose3d> p3d = Optional.of(new Pose3d(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(45))));
    // OperatorC.start().whileTrue(new CMD_RelativeDriveToTarget(limelight, drivetrain)).onFalse(new InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
    // OperatorC.back().whileTrue(new CMD_AbsoluteDriveToTarget(drivetrain, Optional.of(op))).onFalse(new InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autos.getSelectedAuto();
  }

  public void robotPeriodic(){

    SmartDashboard.putNumber("Current RPM", shooter.getFlywheelRPM());
    SmartDashboard.putNumber("Current Setpoint RPM", shooter.MANUAL_RPM);
    SmartDashboard.putNumber("Current Shooter Angle (Degrees)", pivot.calculateDegreesRotation());

    Pose2d visionPose = limelight.getPose();
    // if (!visionPose.equals(new Pose2d())){
    //   // Check if vision pose is within one meter of the current estiamted pose 
    //   // to avoid abnormalities with vision (detecting a tag that isn't present) from
    //   // affecting the accuracy of our pose measurement.
    //   Transform2d t2d = visionPose.minus(drivetrain.getPose());
    //   double dist = Math.sqrt(Math.pow(t2d.getX(), 2) + Math.pow(t2d.getY(), 2));
    //   if (dist <= 1){
    //   }
    // }
    double latencySec = limelight.getCaptureLatency() + limelight.getPipelineLatency();
    drivetrain.addVisionMeasurement(visionPose, latencySec/1000);
  }
}
