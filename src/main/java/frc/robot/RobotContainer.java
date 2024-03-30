// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.PhotonVision.Camera;
import frc.robot.commands.Limelight.CMD_TeleopAimOnDist;
import frc.robot.commands.Limelight.CMD_AlignSource;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_LEDs.BlinkinPattern;
import frc.robot.subsystems.Vision.SUB_Limelight;
import frc.robot.subsystems.Vision.SUB_PhotonVision;
import frc.robot.subsystems.Vision.NoteDetect.NoteVision;
import frc.robot.subsystems.Vision.NoteDetect.NoteVisionIOPhotonVision;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.utils.AutoSelector;
import frc.robot.subsystems.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj.Timer;

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

  public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  public static SUB_Shooter shooter = SUB_Shooter.getInstance();
  public static SUB_Index index = SUB_Index.getInstance();
  public static SUB_Intake intake = SUB_Intake.getInstance();
  public static SUB_Pivot pivot = SUB_Pivot.getInstance();
  public static SUB_Limelight limelight = SUB_Limelight.getInstance();
  public static SUB_LEDs led = new SUB_LEDs(9);
  public static SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
  public static NoteVision noteVision = NoteVision.getInstance();
  public static SUB_Climber climber = new SUB_Climber();

  public static CommandXboxController Driver1 = new CommandXboxController(OIConstants.kDriver1ontrollerPort);
  public static CommandXboxController Driver2 = new CommandXboxController(OIConstants.kDriver2ControllerPort);

  public static AutoSelector autoSelector = new AutoSelector(Driver1);

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
    // new InstantCommand(()->pivot.goToAngle(Pivot.kSideSP)),
    // new RunCommand(()->shooter.shootFlywheelOnRPM(1000), shooter),
    // new SequentialCommandGroup(
    // new WaitCommand(.75),
    // new RunCommand(()->index.setMotorSpeed(0.45), index)
    // )
    // )).onFalse(new SequentialCommandGroup(
    // new InstantCommand(()->index.setMotorSpeed(0)),
    // new InstantCommand(()->SUB_LEDs.ledValue =
    // BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value),
    // new RunCommand(()->shooter.shootFlywheelOnRPM(3000),
    // shooter).withTimeout(0.25)
    // ));

    // Driver1.back().whileTrue(
    //   new ParallelCommandGroup(
    //     new ParallelCommandGroup(
    //         new InstantCommand(() -> pivot.goToAngle(75)),
    //         new InstantCommand(() -> index.starttimer()),
    //         new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
    //         new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
    //             () -> index.CurrentLimitSpike())
    //         .andThen(
    //             new InstantCommand(() -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
    //             new InstantCommand(() -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)))
    //         .andThen(
    //              new InstantCommand(()->intake.setHasNote(true)),
    //             new RunCommand(() -> index.setMotorSpeed(0.0)).withTimeout(0.0).andThen(
    //                 new ParallelCommandGroup(
    //                     new InstantCommand(() -> index.setMotorSpeed(0)),
    //                     new InstantCommand(() -> shooter.setMotorSpeed(0)),
    //                     new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value)))),
    //     NoteVision.getInstance().autoIntake(()-> 0.2, drivetrain, intake)
    //   )  
    // );

    Driver1.start().whileTrue(new ParallelCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(Pivot.kSideSP)),
        new RunCommand(() -> shooter.shootFlywheelOnRPM(2500), shooter),
        new WaitUntilCommand(() -> shooter.getFlywheelRPM() > 2350).andThen(
            new RunCommand(()->index.setMotorSpeed(0.5)).withTimeout(1.0)
        ).andThen(
            new InstantCommand(()->intake.setHasNote(false))
        )

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
                new InstantCommand(()->intake.setHasNote(false)),
                new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)))
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
                    new InstantCommand(()->intake.setHasNote(true)),
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
                     new InstantCommand(()->intake.setHasNote(true)),
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
                new InstantCommand(()->intake.setHasNote(false)),
                new InstantCommand(() -> shooter.shootFlywheelOnRPM(0), shooter))); // Spin Shooter OUT

    Driver2.back().whileTrue(
        new ParallelCommandGroup(
            new RunCommand(() -> shooter.shootFlywheelOnRPM(SUB_Shooter.SetpointRPM), shooter),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= SUB_Shooter.SetpointRPM - 150),
                new RunCommand(() -> index.setMotorSpeed(0.5), index),
                new InstantCommand(()->intake.setHasNote(false))))
    ).onFalse(
        new ParallelCommandGroup(
            new InstantCommand(()->intake.setHasNote(false)),
                    new InstantCommand(() -> index.setMotorSpeed(0)),
                    new InstantCommand(() -> shooter.setMotorSpeed(0))
        )
    );


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
                 new InstantCommand(()->intake.setHasNote(true)),
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
    photonPoseUpdate();
    // limelightPoseUpdate();
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
      drivetrain.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - latencySec / 1000);

    }
    // drivetrain.limelightVisionUpdate(); 
  }

  public static void photonPoseUpdate() {
    Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getEstimatedGlobalPose(drivetrain.getPose());

    
    if (photonPoseOptional.isPresent()) {
        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
        if (photonPose.getX() >= 0 && photonPose.getX() <= 1655.0 / 100 &&
        photonPose.getY() >= 0
        && photonPose.getY() <= 821.0 / 100) {
        }
        //photonPose.pose.
        SmartDashboard.putNumberArray("PHOTON/Pose", new Double[]{photonPose.toPose2d().getX(), photonPose.toPose2d().getY(), photonPose.toPose2d().getRotation().getDegrees()});
        
        SmartDashboard.putNumberArray("PHOTON/Pose3d", new Double[]{photonPose.getX(), photonPose.getY(), photonPose.getZ(), photonPose.getRotation().getQuaternion().getW(), photonPose.getRotation().getQuaternion().getX(), photonPose.getRotation().getQuaternion().getY(), photonPose.getRotation().getQuaternion().getZ()});
        drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);    
    //photonPose.pose.

    // double rotStddev = Units.degreesToRadians(40.0);
    // Pose2d closestTag = drivetrain.at_field.getTagPose(photonVision.getBestTarget().getFiducialId()).get().toPose2d();
    // Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
    // // distance/4 

    //Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2);
    

    }
  }

// public void updatePoseEstimatorWithVisionBotPose() {
//     SUB_PhotonVision.PosePair ppair = photonVision.getPose2dPhotonvision();
    
//     if (ppair == null){
//         return;
//     }

//     // distance from current pose to vision estimated pose
//     double poseDifference = drivetrain.m_poseEstimator.getEstimatedPosition().getTranslation()
//         .getDistance(ppair.pose.getTranslation());

//     double xyStds;
//     double degStds;
//     // multiple targets detected
//     if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
//         xyStds = 0.5;
//         degStds = 6;
//     }
//     // 1 target with large area and close to estimated pose
//     else if (photonVision.getBestTarget().getArea() > 0.8 && poseDifference < 0.5) {
//         xyStds = 1.0;
//         degStds = 12;
//     }
//     // 1 target farther away and estimated pose is close
//     else if (photonVision.getBestTarget().getArea() > 0.1 && poseDifference < 0.3) {
//     xyStds = 2.0;
//     degStds = 30;
//     }
//     // conditions don't match to add a vision measurement
//     else {
//     return;
//     }

//     m_poseEstimator.setVisionMeasurementStdDevs(
//         VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
//     m_poseEstimator.addVisionMeasurement(visionBotPose.pose2d,
//         Timer.getFPGATimestamp() - visionBotPose.latencySeconds);
//   }
}
