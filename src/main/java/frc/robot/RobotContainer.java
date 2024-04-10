// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Limelight.CMD_TeleopAimOnDist;
import frc.robot.commands.Limelight.CMD_AlignSource;
import frc.robot.commands.Limelight.CMD_CenterOnNote;
import frc.robot.subsystems.SUB_Amp;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_LEDs.BlinkinPattern;
import frc.robot.subsystems.Vision.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.utils.AutoSelector;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
    public static SUB_Shooter shooter = SUB_Shooter.getInstance();
    public static SUB_Index index = SUB_Index.getInstance();
    public static SUB_Intake intake = SUB_Intake.getInstance();
    public static SUB_Pivot pivot = SUB_Pivot.getInstance();
    public static SUB_Amp amp = SUB_Amp.getInstance();
    // public static SUB_Limelight limelight = SUB_Limelight.getInstance();
    public static SUB_LEDs led = new SUB_LEDs(9);
    public static SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
    public static SUB_Climber climber = new SUB_Climber();

    public static CommandXboxController Driver1 = new CommandXboxController(OIConstants.kDriver1ontrollerPort);
    public static CommandXboxController Driver2 = new CommandXboxController(OIConstants.kDriver2ControllerPort);

    public static SendableChooser<Boolean> standardPosChecker = new SendableChooser<Boolean>();

    public static AutoSelector autoSelector = new AutoSelector(Driver1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        standardPosChecker.addOption("Odometery Init", Boolean.TRUE);
        standardPosChecker.setDefaultOption("ATag Init", Boolean.FALSE);
        SmartDashboard.putData(standardPosChecker);
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

        pivot.setDefaultCommand(new RunCommand(() -> pivot.runAutomatic(), pivot));
        climber.setDefaultCommand(new RunCommand(() -> {
            climber.runLeft(0);
            climber.runRight(0);
        }, climber));
        led.setDefaultCommand(new RunCommand(() -> led.set(SUB_LEDs.ledValue), led));

        // pivot.set

        // Left Climber Down
        Driver1.leftTrigger().whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> climber.runLeft(Climber.kDownSpeed), climber),
                        new InstantCommand(() -> pivot.goToAngle(Pivot.kLowAngleSP - 5))));

        // Left Climber Up
        Driver1.leftBumper().whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> climber.runLeft(Climber.kUpSpeed), climber),
                        new InstantCommand(() -> pivot.goToAngle(Pivot.kLowAngleSP - 5))));

        // Right Climber Down
        Driver1.rightTrigger().whileTrue(new ParallelCommandGroup(
                new RunCommand(() -> climber.runRight(Climber.kDownSpeed), climber),
                new InstantCommand(() -> pivot.goToAngle(Pivot.kLowAngleSP - 5))));

        // Right Climber Up
        Driver1.rightBumper().whileTrue(new ParallelCommandGroup(
                new RunCommand(() -> climber.runRight(Climber.kUpSpeed), climber),
                new InstantCommand(() -> pivot.goToAngle(Pivot.kLowAngleSP - 5))));

        // Zero Heading
        Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        // Middle Setpoint
        Driver1.povRight().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> pivot.goToAngle(Pivot.kLowMidAngleSP))));// Shoot From Middle Setpoint

        // Top Setpoint
        Driver1.povUp().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> pivot.goToAngle(Constants.Pivot.kSpeakerAngleSP))));// Shoot From Up Close
                                                                                             // Setpoint

        // Bottom Setpoint
        Driver1.povLeft().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> pivot.goToAngle(Pivot.kLowAngleSP - 5))));// Shoot From Bottom Setpoint

        // Lob Shot
        Driver1.start().whileTrue(new ParallelCommandGroup(
                new InstantCommand(() -> pivot.goToAngle(Pivot.kSideSP)),
                new RunCommand(() -> shooter.shootFlywheelOnRPM(2500), shooter),
                new WaitUntilCommand(() -> shooter.getFlywheelRPM() > 2350).andThen(
                        new RunCommand(() -> index.setMotorSpeed(0.5)).withTimeout(1.0)).andThen(
                                new InstantCommand(() -> intake.setHasNote(false)))))
                .onFalse(new SequentialCommandGroup(
                        new InstantCommand(() -> index.setMotorSpeed(0)),
                        new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)));

        // Align to source
        Driver1.a().whileTrue(
                new CMD_AlignSource(pivot, drivetrain, Driver1));

        // Manual Auto shot
        Driver1.b().whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> shooter.shootFlywheelOnRPM(4000), shooter),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= 3500),
                                new RunCommand(() -> index.setMotorSpeed(0.5), index),
                                new InstantCommand(() -> intake.setHasNote(false)),
                                new InstantCommand(
                                        () -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value))))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> shooter.setMotorSpeed(0)))); // Spin Shooter OUT

        // Align to Source Intake
        Driver1.y().whileTrue(new ParallelCommandGroup(
                new CMD_AlignSource(pivot, drivetrain, Driver1),
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
                                        .andThen(new InstantCommand(
                                                () -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value)))))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> shooter.shootFlywheelOnRPM(1500))));

        // Robot relative drive
        Driver1.povDown().whileTrue(
                new RunCommand(
                        () -> drivetrain.drive(
                                -MathUtil.applyDeadband(
                                        Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                        Math.copySign(Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(Driver1.getRawAxis(4), OIConstants.kDriveDeadband),
                                false, true),
                        drivetrain));

        /*==================*\
             Driver Two
        \*==================*/

        // Manual setpoint shooter Up and Down
        Driver2.leftBumper().onTrue(new InstantCommand(() -> SUB_Shooter.MANUAL_RPM -= 250)); // Decrease manual RPM by
                                                                                              // 250
        Driver2.rightBumper().onTrue(
                new InstantCommand(() -> SUB_Shooter.MANUAL_RPM += 250)); // Increase manual RPM by 250

        // Manual Source intake
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
                                        new InstantCommand(() -> intake.setHasNote(true)),
                                        new InstantCommand(() -> index.setMotorSpeed(0)),
                                        new InstantCommand(() -> shooter.setMotorSpeed(0))).andThen(
                                                new SequentialCommandGroup(
                                                        new WaitCommand(.5),
                                                        new ParallelCommandGroup(
                                                                new InstantCommand(
                                                                        () -> Driver1.getHID().setRumble(
                                                                                GenericHID.RumbleType.kBothRumble, 0)),
                                                                new InstantCommand(
                                                                        () -> Driver2.getHID().setRumble(
                                                                                GenericHID.RumbleType.kBothRumble, 0)),
                                                                new InstantCommand(
                                                                        () -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value))))))
                .onFalse(
                        new ParallelCommandGroup(
                                new RunCommand(() -> shooter.setMotorSpeed(0.0), shooter),
                                new RunCommand(() -> index.setMotorSpeed(0.0), index))); // Spin Shooter IN

        // Auto Aim shot
        Driver2.b().whileTrue(
                new ParallelCommandGroup(
                        new ParallelCommandGroup(
                                new RunCommand(() -> shooter.shootFlywheelOnRPM(4000), shooter),
                                new CMD_TeleopAimOnDist(pivot, drivetrain, Driver1)),
                        new InstantCommand(() -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> intake.setHasNote(false)),
                                new InstantCommand(() -> shooter.shootFlywheelOnRPM(0), shooter))); // Spin Shooter OUT

        // Spin manual shooter
        Driver2.back().whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> shooter.shootFlywheelOnRPM(SUB_Shooter.SetpointRPM), shooter),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= SUB_Shooter.SetpointRPM - 150),
                                new RunCommand(() -> index.setMotorSpeed(0.5), index),
                                new InstantCommand(() -> intake.setHasNote(false)))))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> intake.setHasNote(false)),
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> shooter.setMotorSpeed(0))));

        // Intake button
        Driver2.a().whileTrue(
                new ParallelCommandGroup(
                        new InstantCommand(() -> pivot.goToAngle(75)),
                        new InstantCommand(() -> index.starttimer()),
                        new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
                        new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
                                () -> index.CurrentLimitSpike())
                        .andThen(
                                new InstantCommand(
                                        () -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                                new InstantCommand(
                                        () -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)))
                        .andThen(
                                new InstantCommand(() -> intake.setHasNote(true)),
                                new RunCommand(() -> index.setMotorSpeed(0.0)).withTimeout(0.0).andThen(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                                new InstantCommand(() -> shooter.setMotorSpeed(0)),
                                                new InstantCommand(
                                                        () -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value)))))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> intake.setMotorSpeed(0)),
                                new InstantCommand(() -> shooter.shootFlywheelOnRPM(1500))).andThen(
                                        new SequentialCommandGroup(
                                                new WaitCommand(.5),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(() -> Driver1.getHID()
                                                                .setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                                                        new InstantCommand(() -> Driver2.getHID()
                                                                .setRumble(GenericHID.RumbleType.kBothRumble, 0))),
                                                new WaitCommand(1.0),
                                                new InstantCommand(
                                                        () -> pivot.goToAngle(Constants.Pivot.kLowAngleSP)))));

        // Outtake button
        Driver2.x().whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> index.setMotorSpeed(-0.6), index),
                        new RunCommand(() -> intake.setMotorSpeed(-0.6), intake)))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> intake.setMotorSpeed(0))));

        // Drive Intake OUT
        Driver2.leftTrigger()
                .whileTrue(new RunCommand(() -> intake.setMotorSpeed(-Constants.Intake.kOutakeSpeed), intake))
                .onFalse(new InstantCommand(() -> intake.setMotorSpeed(0.0)));

        // Drive Pivot Down Manual
        Driver2.povRight().whileTrue(new RunCommand(() -> pivot.runManual(-0.2), pivot));

        // Drive Pivot Up Manual
        Driver2.povLeft().whileTrue(new RunCommand(() -> pivot.runManual(0.2), pivot));

        // Amp Scoring Seq
        Driver2.y().whileTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> pivot.goToAngle(75.56)),
                        new InstantCommand(() -> amp.starttimer()),
                        new ParallelCommandGroup(
                                new RunCommand(() -> amp.setMotorSpeed(0.7)).until(
                                        () -> amp.CurrentLimitSpike())
                                        .andThen(new InstantCommand(() -> amp.setMotorSpeed(0.0))),
                                new RunCommand(() -> shooter.shootFlywheelOnRPM(2000), shooter),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= 1500),
                                        new RunCommand(() -> index.setMotorSpeed(0.5), index),
                                        new InstantCommand(() -> intake.setHasNote(false)),
                                        new InstantCommand(
                                                () -> SUB_LEDs.ledValue = BlinkinPattern.RAINBOW_RAINBOW_PALETTE.value)))))
                .onFalse(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.goToAngle(75.56)),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> amp.starttimer()),
                                        new RunCommand(() -> amp.setMotorSpeed(-0.7)).until(
                                                () -> amp.CurrentLimitSpike())
                                                .andThen(new InstantCommand(() -> amp.setMotorSpeed(0.0))),
                                        new InstantCommand(() -> index.setMotorSpeed(0)),
                                        new InstantCommand(() -> shooter.setMotorSpeed(0)))));

        // Center on Note Pickup
        Driver1.x().whileTrue(
                new ParallelCommandGroup(
                        new CMD_CenterOnNote(drivetrain, photonVision, Driver1).withTimeout(1.5).andThen(
                                new RunCommand(() -> drivetrain.drive(-0.5, 0, 0, false, true))).withTimeout(3.0)
                                .until(() -> index.CurrentLimitSpike()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> pivot.goToAngle(75)),
                                new InstantCommand(() -> index.starttimer()),
                                new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
                                new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
                                        () -> index.CurrentLimitSpike())
                                .andThen(
                                        new InstantCommand(
                                                () -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                                        new InstantCommand(
                                                () -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)))
                                .andThen(
                                        new InstantCommand(() -> intake.setHasNote(true)),
                                        new RunCommand(() -> index.setMotorSpeed(0.0)).withTimeout(0.0).andThen(
                                                new ParallelCommandGroup(
                                                        new InstantCommand(() -> index.setMotorSpeed(0)),
                                                        new InstantCommand(() -> shooter.setMotorSpeed(0)),
                                                        new InstantCommand(
                                                                () -> SUB_LEDs.ledValue = BlinkinPattern.GREEN.value))))))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> intake.setMotorSpeed(0)),
                                new InstantCommand(() -> shooter.shootFlywheelOnRPM(4000))).andThen(
                                        new SequentialCommandGroup(
                                                new WaitCommand(.5),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(() -> Driver1.getHID()
                                                                .setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                                                        new InstantCommand(() -> Driver2.getHID()
                                                                .setRumble(GenericHID.RumbleType.kBothRumble, 0))),
                                                new WaitCommand(1.0),
                                                new InstantCommand(
                                                        () -> pivot.goToAngle(Constants.Pivot.kLowAngleSP)))));

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
        teleopPhotonPose();
    }

    public void robotPeriodic() {

        // COMMENT OUT BELOW IF IT DOESN'T WORK, ALSO IN SUB_PHOTONVISION
        if (photonVision.getBestNote() != null && photonVision.hasResults) {
            SmartDashboard.putNumber("NOTE/YAW", photonVision.getBestNote().getYaw());
        }

    }

    public static void photonPoseUpdate() {
        Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getEstimatedGlobalPose(drivetrain.getPose());

        if (photonPoseOptional.isPresent()) {
            Pose3d photonPose = photonPoseOptional.get().estimatedPose;
            if (photonPose.getX() >= 0 && photonPose.getX() <= 1655.0 / 100 &&
                    photonPose.getY() >= 0
                    && photonPose.getY() <= 821.0 / 100) {
            }

            if (photonVision.getBestTarget() == null) {
                return;
            }
            double rotStddev = Units.degreesToRadians(70.0);
            Pose2d closestTag = photonVision.at_field.getTagPose(photonVision.getBestTarget().getFiducialId()).get()
                    .toPose2d();
            Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
            // distance/4
            double distance = Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));
            double sqDistance = (Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));

            // if (distance <= 5){
            double xStddev = distance / 1.0;
            double yStddev = xStddev * 4;

            // if(translate.getX() > 8){
            // xStddev = sqDistance/0.1;
            // }

            drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));
            // } else {
            // double xStddev = sqDistance/2.0;
            // double yStddev = xStddev * 7;

            // if(translate.getX() > 8){
            // xStddev = sqDistance/0.1;
            // }
            // drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev,
            // yStddev, rotStddev));
            // }

            SmartDashboard.putNumberArray("PHOTON/Pose", new Double[] { photonPose.toPose2d().getX(),
                    photonPose.toPose2d().getY(), photonPose.toPose2d().getRotation().getDegrees() });
            SmartDashboard.putNumberArray("PHOTON/Pose3d", new Double[] { photonPose.getX(), photonPose.getY(),
                    photonPose.getZ(), photonPose.getRotation().getQuaternion().getW(),
                    photonPose.getRotation().getQuaternion().getX(), photonPose.getRotation().getQuaternion().getY(),
                    photonPose.getRotation().getQuaternion().getZ() });
            drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
        }
    }

    public static void teleopPhotonPose() {
        Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getEstimatedGlobalPose(drivetrain.getPose());

        if (photonPoseOptional.isPresent()) {
            Pose3d photonPose = photonPoseOptional.get().estimatedPose;
            if (photonPose.getX() >= 0 && photonPose.getX() <= 1655.0 / 100 &&
                    photonPose.getY() >= 0
                    && photonPose.getY() <= 821.0 / 100) {
            }

            if (photonVision.getBestTarget() == null) {
                return;
            }
            double rotStddev = Units.degreesToRadians(70.0);
            Pose2d closestTag = photonVision.at_field.getTagPose(photonVision.getBestTarget().getFiducialId()).get()
                    .toPose2d();
            Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
            // distance/4
            double distance = Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));
            double sqDistance = (Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));

            // if (distance <= 5){
            double xStddev = distance / 16.0;
            double yStddev = xStddev * 4;

            // if(translate.getX() > 8){
            // xStddev = sqDistance/0.1;
            // }

            drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));
            // } else {
            // double xStddev = sqDistance/2.0;
            // double yStddev = xStddev * 7;

            // if(translate.getX() > 8){
            // xStddev = sqDistance/0.1;
            // }
            // drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev,
            // yStddev, rotStddev));
            // }

            SmartDashboard.putNumberArray("PHOTON/Pose", new Double[] { photonPose.toPose2d().getX(),
                    photonPose.toPose2d().getY(), photonPose.toPose2d().getRotation().getDegrees() });
            SmartDashboard.putNumberArray("PHOTON/Pose3d", new Double[] { photonPose.getX(), photonPose.getY(),
                    photonPose.getZ(), photonPose.getRotation().getQuaternion().getW(),
                    photonPose.getRotation().getQuaternion().getX(), photonPose.getRotation().getQuaternion().getY(),
                    photonPose.getRotation().getQuaternion().getZ() });
            drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
        }
    }
}
