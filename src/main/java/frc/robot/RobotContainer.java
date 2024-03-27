// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Limelight.CMD_TeleopAimOnDist;
import frc.robot.commands.Limelight.CMD_AlignSource;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_LEDs.BlinkinPattern;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.utils.AutoSelector;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_PhotonVision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
  public static SUB_LEDs led = new SUB_LEDs(9);
  //public static SUB_PhotonVision photonVision = new SUB_PhotonVision();
  public static SUB_Climber climber = new SUB_Climber();

  public static CommandXboxController Driver1 = new CommandXboxController(OIConstants.kDriver1ontrollerPort);
  public static CommandXboxController Driver2 = new CommandXboxController(OIConstants.kDriver2ControllerPort);

  public static AutoSelector autoSelector = new AutoSelector(drivetrain, index, intake, shooter, pivot, limelight,
      Driver1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(Driver1.getRawAxis(1),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Driver1.getRawAxis(0),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Driver1.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
            drivetrain));

    shooter.setDefaultCommand(new RunCommand(() -> shooter.shootFlywheelOnRPM(0), shooter));
    // index.setDefaultCommand(new RunCommand(()->index.setMotorSpeed(0), index));
    // intake.setDefaultCommand(new RunCommand(()->intake.setMotorSpeed(0),
    // intake));
    pivot.setDefaultCommand(new RunCommand(() -> pivot.runAutomatic(), pivot));
    climber.setDefaultCommand(new RunCommand(() -> {
      climber.runLeft(0);
      climber.runRight(0);
    }, climber));

    shooter.setDefaultCommand(new RunCommand(() -> shooter.shootFlywheelOnRPM(0), shooter));
    // index.setDefaultCommand(new RunCommand(()->index.setMotorSpeed(0), index));
    // intake.setDefaultCommand(new RunCommand(()->intake.setMotorSpeed(0),
    // intake));
    pivot.setDefaultCommand(new RunCommand(() -> pivot.runAutomatic(), pivot));
    climber.setDefaultCommand(new RunCommand(() -> {
      climber.runLeft(0);
      climber.runRight(0);
    }, climber));
    led.setDefaultCommand(new RunCommand(() -> led.set(SUB_LEDs.ledValue), led));

    // pivot.set

    Driver1.leftTrigger().whileTrue(new RunCommand(() -> climber.runLeft(Climber.kDownSpeed), climber));
    Driver1.leftBumper().whileTrue(new RunCommand(() -> climber.runLeft(Climber.kUpSpeed), climber));

    Driver1.rightTrigger().whileTrue(new RunCommand(() -> climber.runRight(Climber.kDownSpeed), climber));
    Driver1.rightBumper().whileTrue(new RunCommand(() -> climber.runRight(Climber.kUpSpeed), climber));

    Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    Driver1.povRight().onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(75))));// Shoot From Middle Setpoint

    Driver1.povUp().onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(Constants.Pivot.kSpeakerAngleSP))));// Shoot From Up Close Setpoint

    Driver1.povDown().onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(50))));// Shoot From Bottom Setpoint

    // Driver1.start().whileTrue( new ParallelCommandGroup(
    // //new InstantCommand(()->pivot.goToAngle(Pivot.kSideSP)),
    // new RunCommand(()->shooter.shootFlywheelOnRPM(1000), shooter),
    // new SequentialCommandGroup(
    // new WaitCommand(.75),
    // new RunCommand(()->index.setMotorSpeed(0.45), index)
    // )
    // )).onFalse(new SequentialCommandGroup(
    // new InstantCommand(()->index.setMotorSpeed(0)),
    // new InstantCommand(()->SUB_LEDs.ledValue =
    // BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)
    // //new RunCommand(()->shooter.shootFlywheelOnRPM(3000),
    // shooter).withTimeout(0.25)
    // ));

    Driver1.start().whileTrue(new ParallelCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(Pivot.kSideSP)),
        new RunCommand(() -> shooter.shootFlywheelOnRPM(3000), shooter),
        new WaitUntilCommand(() -> shooter.getFlywheelRPM() > 2750).andThen()

    )).onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> index.setMotorSpeed(0)),
        new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)
    // new RunCommand(()->shooter.shootFlywheelOnRPM(3000),
    // shooter).withTimeout(0.25)
    ));

    Driver1.back().whileTrue(new InstantCommand(() -> pivot.goToAngle(Pivot.kSideSP)));

    Driver1.a().whileTrue(
        new CMD_AlignSource(pivot, limelight, drivetrain, Driver1));

    Driver1.b().whileTrue(
        new ParallelCommandGroup(
            new RunCommand(() -> shooter.shootFlywheelOnRPM(4000), shooter),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= 3500),
                new RunCommand(() -> index.setMotorSpeed(0.5), index),
                new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)))// .andThen(new
                                                                                                            // InstantCommand(()->SUB_LEDs.ledValue
                                                                                                            // =
                                                                                                            // BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value))
    ).onFalse(
        new ParallelCommandGroup(
            new InstantCommand(() -> index.setMotorSpeed(0)),
            new InstantCommand(() -> shooter.setMotorSpeed(0)))); // Spin Shooter OUT

    Driver1.y().whileTrue(new ParallelCommandGroup(
        new CMD_AlignSource(pivot, limelight, drivetrain, Driver1),
        new ParallelCommandGroup(
            new InstantCommand(() -> index.starttimer()),
            new RunCommand(() -> index.setMotorSpeed(-Constants.Intake.kIndexSpeed), index),
            new RunCommand(() -> shooter.shootFlywheelOnRPM(-1000), shooter)).until(
                () -> index.CurrentLimitSpike())
            .andThen(
                new RunCommand(() -> index.setMotorSpeed(-0.05)).withTimeout(0.025))
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(() -> index.setMotorSpeed(0)),
                    new InstantCommand(() -> shooter.setMotorSpeed(0)))
                    .andThen(new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value)))))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> index.setMotorSpeed(0)),
                new InstantCommand(() -> shooter.shootFlywheelOnRPM(1500))));

    Driver1.povLeft().whileTrue(
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.copySign(Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Driver1.getRawAxis(4), OIConstants.kDriveDeadband),
                false, true),
            drivetrain));

    /*
     * ================== *\
     * Driver Two
     * \* ==================
     */
    Driver2.leftBumper().onTrue(new InstantCommand(() -> SUB_Shooter.MANUAL_RPM -= 250)); // Decrease manual RPM by 250

    Driver2.rightBumper().onTrue(
        new InstantCommand(() -> SUB_Shooter.MANUAL_RPM += 250)); // Increase manual RPM by 250

    Driver2.rightTrigger().whileTrue(
        new ParallelCommandGroup(
            new InstantCommand(() -> index.starttimer()),
            new RunCommand(() -> index.setMotorSpeed(-Constants.Intake.kIndexSpeed), index),
            new RunCommand(() -> shooter.shootFlywheelOnRPM(-1000), shooter)).until(
                () -> index.CurrentLimitSpike())
            .andThen(
                new RunCommand(() -> index.setMotorSpeed(-0.05)).withTimeout(0.025))
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(() -> index.setMotorSpeed(0)),
                    new InstantCommand(() -> shooter.setMotorSpeed(0))).andThen(
                        new SequentialCommandGroup(
                            new WaitCommand(.5),
                            new ParallelCommandGroup(
                                new InstantCommand(
                                    () -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                                new InstantCommand(
                                    () -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                                new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value))))))
        .onFalse(
            new ParallelCommandGroup(
                new RunCommand(() -> shooter.setMotorSpeed(0.0), shooter),
                new RunCommand(() -> index.setMotorSpeed(0.0), index))); // Spin Shooter IN

    Driver2.rightTrigger().whileTrue(
        new ParallelCommandGroup(
            new InstantCommand(() -> index.starttimer()),
            new RunCommand(() -> index.setMotorSpeed(-Constants.Intake.kIndexSpeed), index),
            new RunCommand(() -> shooter.shootFlywheelOnRPM(-1000), shooter)).until(
                () -> index.CurrentLimitSpike())
            .andThen(
                new RunCommand(() -> index.setMotorSpeed(-0.05)).withTimeout(0.045))
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(() -> index.setMotorSpeed(0)),
                    new InstantCommand(() -> shooter.setMotorSpeed(0)))))
        .onFalse(
            new ParallelCommandGroup(
                new RunCommand(() -> shooter.setMotorSpeed(0.0), shooter),
                new RunCommand(() -> index.setMotorSpeed(0.0), index))); // Spin Shooter IN

    Driver2.b().whileTrue(
        new ParallelCommandGroup(
            new ParallelCommandGroup(
                new RunCommand(() -> shooter.shootFlywheelOnRPM(4000), shooter),
                new CMD_TeleopAimOnDist(pivot, limelight, drivetrain, Driver1)),
            new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> shooter.shootFlywheelOnRPM(0), shooter))); // Spin Shooter OUT

    Driver2.a().whileTrue(
        new ParallelCommandGroup(
            new InstantCommand(() -> pivot.goToAngle(75)),
            new InstantCommand(() -> index.starttimer()),
            new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
            new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
                () -> index.CurrentLimitSpike())
            .andThen(
                new InstantCommand(() -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                new InstantCommand(() -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)))
            .andThen(
                new RunCommand(() -> index.setMotorSpeed(0.0)).withTimeout(0.0).andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> index.setMotorSpeed(0)),
                        new InstantCommand(() -> shooter.setMotorSpeed(0)),
                        new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value)))))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> index.setMotorSpeed(0)),
                new InstantCommand(() -> intake.setMotorSpeed(0)),
                new InstantCommand(() -> shooter.shootFlywheelOnRPM(1500))).andThen(
                    new SequentialCommandGroup(
                        new WaitCommand(.5),
                        new ParallelCommandGroup(
                            new InstantCommand(() -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                            new InstantCommand(() -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0))),
                        new WaitCommand(1.0),
                        new InstantCommand(() -> pivot.goToAngle(Constants.Pivot.kLowAngleSP)))));

    // Driver2.start().whileTrue(new RunCommand(()->led.set(0.5), led)).onFalse(new
    // RunCommand(()->led.set(0.0), led));

    // Driver2.a().whileTrue(
    // new ParallelCommandGroup(
    // new InstantCommand(()->pivot.goToAngle(Pivot.kLowMidAngleSP)),
    // new RunCommand(()->shooter.setMotorSpeed(0), shooter),
    // new RunCommand(()->index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
    // new RunCommand(()->intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))
    // .until(()->index.getTopBannerSensor())
    // .andThen(new ParallelCommandGroup(
    // new RunCommand(()->index.setMotorSpeed(0.0))
    // ))
    // )).onFalse(new ParallelCommandGroup(

    // new InstantCommand(()->index.setMotorSpeed(0)),
    // new InstantCommand(()->intake.setMotorSpeed(0))
    // )); // Suspicious if it will work or not, if it doesn't, just put onTrue();

    Driver2.x().whileTrue(
        new ParallelCommandGroup(
            new RunCommand(() -> index.setMotorSpeed(-0.6), index),
            new RunCommand(() -> intake.setMotorSpeed(-0.6), intake)))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> index.setMotorSpeed(0)),
                new InstantCommand(() -> intake.setMotorSpeed(0))));

    // Driver2.a().whileTrue(new RunCommand(()->drivetrain.;, null) );
    Driver2.leftTrigger().whileTrue(new RunCommand(() -> intake.setMotorSpeed(-Constants.Intake.kOutakeSpeed), intake))
        .onFalse(new InstantCommand(() -> intake.setMotorSpeed(0.0))); // Drive Intake OUT
    Driver2.povRight().whileTrue(new RunCommand(() -> pivot.runManual(-0.2), pivot));
    Driver2.povLeft().whileTrue(new RunCommand(() -> pivot.runManual(0.2), pivot));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Pose3d op3d = drivetrain.at_field.getTagPose(4).get();
    // Pose3d op = new Pose3d(new Pose2d(op3d.getX()-3, op3d.getY()-1,
    // Rotation2d.fromDegrees(0)));
    // Optional<Pose3d> p3d = Optional.of(new Pose3d(new Pose2d(0.5, 0.5,
    // Rotation2d.fromDegrees(45))));
    // Driver2.start().whileTrue(new CMD_RelativeDriveToTarget(limelight,
    // drivetrain)).onFalse(new
    // InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
    // Driver2.back().whileTrue(new CMD_AbsoluteDriveToTarget(drivetrain,
    // Optional.of(op))).onFalse(new
    // InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void teleopPeriodic() {
    // photonPoseUpdate();

    limelightPoseUpdate();
  }

  public void robotPeriodic() {

  }

  public void limelightPoseUpdate() {
    Pose2d visionPose = limelight.getPose();

    SmartDashboard.putNumber("LL X pose", visionPose.getX());
    SmartDashboard.putNumber("LL Y pose", visionPose.getY());
    if (!visionPose.equals(new Pose2d()) &&
        visionPose.getX() >= 0 && visionPose.getX() <= 1655.0 / 100 &&
        visionPose.getY() >= 0 && visionPose.getY() <= 821.0 / 100) {
      // Check if vision pose is within one meter of the current estiamted pose
      // to avoid abnormalities with vision (detecting a tag that isn't present) from
      // affecting the accuracy of our pose measurement.
      Transform2d t2d = visionPose.minus(drivetrain.getPose());
      double dist = Math.sqrt(Math.pow(t2d.getX(), 2) + Math.pow(t2d.getY(), 2));
      double latencySec = limelight.getCaptureLatency() + limelight.getPipelineLatency();
      drivetrain.addVisionMeasurement(visionPose, latencySec / 1000);

    }
    // drivetrain.limelightVisionUpdate(); 
  }

  public void photonPoseUpdate() {
    // SUB_PhotonVision.PosePair photonPose = photonVision.getPose2dPhotonvision();
    // if (photonPose != null) {
    // if (photonPose.pose.getX() >= 0 && photonPose.pose.getX() <= 1655.0 / 100 &&
    // photonPose.pose.getY() >= 0
    // && photonPose.pose.getY() <= 821.0 / 100) {
    // }
    // drivetrain.addVisionMeasurement(photonPose.pose, photonPose.time);
    // }
    // SmartDashboard.putNumber("Current RPM", shooter.getFlywheelRPM());
    // SmartDashboard.putNumber("Current Setpoint RPM", shooter.MANUAL_RPM);
    // SmartDashboard.putNumber("Current Shooter Angle (Degrees)",
    // pivot.calculateDegreesRotation());

    // SmartDashboard.putNumber("X Pose", drivetrain.getPose().getX());
    // SmartDashboard.putNumber("Y Pose", drivetrain.getPose().getY());

  }
}
