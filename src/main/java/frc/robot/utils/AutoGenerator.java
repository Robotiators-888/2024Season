package frc.robot.utils;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoActions.CMD_Shoot;
import frc.robot.commands.AutoActions.CMD_ShootSEQ;
import frc.robot.commands.Limelight.CMD_AutoAimOnDist;
import frc.robot.commands.Limelight.CMD_AutoCenterOnNote;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.Vision.SUB_Limelight;
import frc.robot.subsystems.Vision.SUB_PhotonVision;

/** This utility class is built for selecting made autos */
public class AutoGenerator {
  private SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  SUB_Index index = SUB_Index.getInstance();
  SUB_Intake intake = SUB_Intake.getInstance();
  SUB_Shooter shooter = SUB_Shooter.getInstance();
  SUB_Pivot pivot = SUB_Pivot.getInstance();
  SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
  // SUB_Limelight limelight = SUB_Limelight.getInstance();
  CommandXboxController driver1;

  public static enum StartPosition {
    SubwooferFront(FieldConstants.subwooferFront),
    SubwooferAmp(FieldConstants.subwooferAmp),
    SubwooferSource(FieldConstants.subwooferSource),
    Amp(new Pose2d(
        new Translation2d(
            1.40,
            6.80),
        Rotation2d.fromDegrees(180))),
    Podium(new Pose2d(
        new Translation2d(
            1.40,
            4.10),
        Rotation2d.fromDegrees(180))),
    Source(new Pose2d(
        new Translation2d(
            1.40,
            3.30),
        Rotation2d.fromDegrees(180)));

    public final Pose2d startPose;

    StartPosition(Pose2d startPose) {
      this.startPose = startPose;
    }
  }

  public AutoGenerator(CommandXboxController driver1) {
    this.driver1 = driver1;

    AutoBuilder
        .configureHolonomic(drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new HolonomicPathFollowerConfig(new PIDConstants(1.5, 0.0, 0.0), new PIDConstants(5.0, 0, 0),
                Constants.Drivetrain.kMaxModuleSpeed, Constants.Drivetrain.kTrackRadius, new ReplanningConfig()),
            AllianceFlipUtil::shouldFlip, drivetrain);

    registerAllCommands();

  }

  // ====================================================================
  //                              Routines
  // ====================================================================

  public Command resetOdometry(Pose2d pose) {
    if (!RobotContainer.standardPosChecker.getSelected().booleanValue()) {
      // drivetrain.getPose().rotateBy(Rotation2d.fromDegrees(180));
      // drivetrain.resetOdometry(
      // new Pose2d(
      // SmartDashboard.getNumber("STARTING POSE/ABSOLUTE X meters", 0),
      // SmartDashboard.getNumber("STARTING POSE/ABSOLUTE Y meters", 0),
      // Rotation2d.fromDegrees(SmartDashboard.getNumber("STARTING POSE/ABSOLUTE
      // ROTATION degrees", 0))));
      return new InstantCommand(
          () -> drivetrain.resetOdometry(drivetrain.getPose()));
    } else {
      return new InstantCommand(
          () -> drivetrain.resetOdometry(pose));
    }
  }

  public Command aimedShot(){
    return new CMD_AutoAimOnDist(SUB_Pivot.getInstance(), SUB_Drivetrain.getInstance()).withTimeout(1.2).andThen(
      new ParallelCommandGroup(
        new RunCommand(() -> shooter.shootFlywheelOnRPM(4000), shooter),
        new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= 3500),
                new RunCommand(() -> index.setMotorSpeed(0.5), index),
                new InstantCommand(() -> intake.setHasNote(false))))
    );
  }

  public Command visionAlignToNote() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
          new CMD_AutoCenterOnNote(drivetrain, photonVision).withTimeout(2.5).andThen(
              new RunCommand(() -> drivetrain.drive(-0.5, 0, 0, false, true))).withTimeout(3.0)
              .until(() -> index.CurrentLimitSpike()),
          new ParallelCommandGroup(
              new InstantCommand(() -> pivot.goToAngle(75)),
              new InstantCommand(() -> index.starttimer()),
              new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
              new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
                  () -> index.CurrentLimitSpike())
              .andThen(
                  new InstantCommand(() -> intake.setHasNote(true)),
                  new RunCommand(() -> index.setMotorSpeed(0.0)).withTimeout(0.0).andThen(
                      new ParallelCommandGroup(
                          new InstantCommand(() -> index.setMotorSpeed(0)),
                          new InstantCommand(() -> shooter.setMotorSpeed(0))))))
          .andThen(
              new ParallelCommandGroup(
                  new InstantCommand(() -> index.setMotorSpeed(0)),
                  new InstantCommand(() -> intake.setMotorSpeed(0)),
                  new InstantCommand(() -> shooter.shootFlywheelOnRPM(4000)))),
        new WaitCommand(0),
        ()->SUB_PhotonVision.getInstance().hasResults);
  }

  public Command resetOdometry(Pose2d pose, Rotation2d rot) {
    pose.rotateBy(new Rotation2d(Math.abs(rot.getDegrees() - pose.getRotation().getDegrees())));
    return new InstantCommand(
        () -> drivetrain.resetOdometry(pose));
  }

  private static Translation2d getFORR(Translation2d pos) {
    return AllianceFlipUtil.apply(FieldConstants.speakerAimPoint).minus(pos);
  }

  public Command aimAtPoint(Translation2d pos, SUB_Drivetrain rotation) {
    return rotation
        .pidControlledHeading(() -> Optional.of(getFORR(pos)).map((t) -> new Rotation2d(t.getX(), t.getY())));
  }

  // TODO: Test SEQ
  public Command scoringSequence() {
    return new RunCommand(() -> shooter.shootFlywheelOnRPM(4500), shooter)
        .until(() -> shooter.getFlywheelRPM() >= 4000)
        .andThen(new RunCommand(() -> index.setMotorSpeed(0.75)).withTimeout(0.10))
        .andThen(new InstantCommand(() -> index.setMotorSpeed(0)))
        .andThen(new InstantCommand(() -> intake.setHasNote(true)));
  }

  public Command scoringSequence(double setpoint, int rpm) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(setpoint)),
        //new WaitCommand(.25),
        new WaitUntilCommand(()->(Math.abs(pivot.getAngle()-setpoint)<=2)),
        new RunCommand(() -> shooter.shootFlywheelOnRPM(rpm), shooter)
            .until(() -> shooter.getFlywheelRPM() >= rpm - 250)
            .andThen(new RunCommand(() -> index.setMotorSpeed(0.75)).withTimeout(0.15))
            .andThen(new InstantCommand(() -> index.setMotorSpeed(0)))
            .andThen(new InstantCommand(() -> intake.setHasNote(true))));
  }

  public Command scoringSequence(double setpoint, int rpm, double delay) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(setpoint)),
        new InstantCommand(() -> pivot.goToAngle(setpoint)),
        new WaitCommand(delay),
        new RunCommand(() -> shooter.shootFlywheelOnRPM(rpm), shooter)
            .until(() -> shooter.getFlywheelRPM() >= rpm - 250)
            .andThen(new RunCommand(() -> index.setMotorSpeed(0.75)).withTimeout(0.15))
            .andThen(new InstantCommand(() -> index.setMotorSpeed(0)))
            .andThen(new InstantCommand(() -> intake.setHasNote(true))));
  }

  public Command autoAimShot(double delay) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new CMD_AutoAimOnDist(pivot, drivetrain).withTimeout(2.0),
            new RunCommand(() -> shooter.shootFlywheelOnRPM(4500), shooter)).andThen(
                new ParallelCommandGroup(
                    new RunCommand(() -> shooter.shootFlywheelOnRPM(4500), shooter),
                    new RunCommand(() -> index.setMotorSpeed(0.5), index)).withTimeout(0.5))
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(() -> intake.setHasNote(true)),
                    new InstantCommand(() -> index.setMotorSpeed(0.0), index))));
  }

  public Command pathIntake(String path) {
    return new ParallelCommandGroup(
        runIntake(),
        PathPlannerBase.followTrajectory(path));
  }

  public Command runShooter(int rpm) {
    return new RunCommand(
        () -> shooter.shootFlywheelOnRPM(rpm), shooter);
  }

  // TODO: Test CMD
  public Command runIntake() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> pivot.goToAngle(75)),
        new InstantCommand(() -> index.starttimer()),
        new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
        new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
            () -> index.CurrentLimitSpike())
        .andThen(
            new InstantCommand(() -> intake.setHasNote(true)),
            new RunCommand(() -> index.setMotorSpeed(0.1)).withTimeout(0.025).andThen(new ParallelCommandGroup(
                new InstantCommand(() -> index.setMotorSpeed(0)),
                new InstantCommand(() -> intake.setMotorSpeed(0)))));
  }

  public Command stopIntake() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> index.setMotorSpeed(0)),
        new InstantCommand(() -> intake.setMotorSpeed(0)));
  }

  public Command stopAll() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> index.setMotorSpeed(0)),
        new InstantCommand(() -> intake.setMotorSpeed(0)),
        new InstantCommand(() -> shooter.setMotorSpeed(1500)));
  }

  public Command setPivotSetpoint(double setpoint) {
    return new InstantCommand(() -> pivot.setPivotSetpoint(setpoint));
  }

  public Command dumpAuto() {
    return new ParallelCommandGroup(
        setPivotSetpoint(Constants.Pivot.kSpeakerAngleSP),
        new CMD_Shoot(shooter, index, drivetrain, intake, pivot));
  }

  // ====================================================================
  //                                  Helpers
  // ====================================================================

  public void registerAllCommands() {
    NamedCommands.registerCommand("RunIntake", runIntake());
    NamedCommands.registerCommand("ScoringSequence", new CMD_Shoot(shooter, index, drivetrain, intake, pivot));
    NamedCommands.registerCommand("StopIntake", stopIntake());
    NamedCommands.registerCommand("StopAll", stopAll());
    NamedCommands.registerCommand("Amp Setpoint", setPivotSetpoint(Constants.Pivot.kSpeakerAngleSP + 5));
    NamedCommands.registerCommand("Launch Setpoint", setPivotSetpoint(Constants.Pivot.kLowMidAngleSP));
    NamedCommands.registerCommand("LowShot Setpoint", setPivotSetpoint(Constants.Pivot.kLowAngleSP - 10));
    NamedCommands.registerCommand("DumpAuto", new CMD_ShootSEQ(shooter, index, pivot));
  }

}
