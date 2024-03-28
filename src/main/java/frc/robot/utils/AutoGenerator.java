package frc.robot.utils;

import static edu.wpi.first.units.Units.anonymous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.AutoActions.CMD_Shoot;
import frc.robot.commands.AutoActions.CMD_ShootSEQ;
import frc.robot.commands.Limelight.CMD_AutoAimOnDist;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.Vision.SUB_Limelight;
import frc.robot.subsystems.Vision.NoteDetect.NoteVision;

/** This utility class is built for selecting made autos */
public class AutoGenerator {
  private SUB_Drivetrain drivetrain;
  SUB_Index index = SUB_Index.getInstance();
  SUB_Intake intake = SUB_Intake.getInstance();
  SUB_Shooter shooter = SUB_Shooter.getInstance(); 
  SUB_Pivot pivot = SUB_Pivot.getInstance();
  SUB_Limelight limelight = SUB_Limelight.getInstance();
  CommandXboxController driver1;

  
  public AutoGenerator(CommandXboxController driver1) {
    this.driver1 = driver1;


    
    AutoBuilder.configureHolonomic(drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds, drivetrain::driveRobotRelative,
     new HolonomicPathFollowerConfig(new PIDConstants(1.5, 0.0,0.0), new PIDConstants(5.0, 0,0), Constants.Drivetrain.kMaxModuleSpeed, Constants.Drivetrain.kTrackRadius, new ReplanningConfig())
    , ()->{
      var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
    }, drivetrain);


    registerAllCommands();
   
    
  }

  // ====================================================================
  //                          Paths
  // ====================================================================


  // ====================================================================
  //                          Routines
  // ====================================================================
  
    public Command resetOdometry(Pose2d pose){
      return new InstantCommand(
        () -> drivetrain.resetOdometry(pose)
      );
    }


  //TODO: Test SEQ
    public Command scoringSequence(){
      return new RunCommand(()->shooter.shootFlywheelOnRPM(4000), shooter)
      .until(()->shooter.getFlywheelRPM() >= 3500)
      .andThen(new RunCommand(()->index.setMotorSpeed(0.5)).withTimeout(0.25))
      .andThen(new InstantCommand(()->index.setMotorSpeed(0)))
      .andThen(new InstantCommand(()->intake.setHasNote(true)));
    }

    public Command scoringSequence(double setpoint, int rpm){
      return new SequentialCommandGroup(
        new InstantCommand(()->pivot.goToAngle(setpoint)),
        new WaitCommand(.25),
        new RunCommand(()->shooter.shootFlywheelOnRPM(rpm), shooter)
      .until(()->shooter.getFlywheelRPM() >= rpm - 500)
      .andThen(new RunCommand(()->index.setMotorSpeed(0.5)).withTimeout(0.25))
      .andThen(new InstantCommand(()->index.setMotorSpeed(0)))
      .andThen(new InstantCommand(()->intake.setHasNote(true)))
      );
    }

    public Command scoringSequence(double setpoint, int rpm, double delay){
      return new SequentialCommandGroup(
        new InstantCommand(()->pivot.goToAngle(setpoint)),
        new InstantCommand(()->pivot.goToAngle(setpoint)),
        new WaitCommand(delay),
        new RunCommand(()->shooter.shootFlywheelOnRPM(rpm), shooter)
      .until(()->shooter.getFlywheelRPM() >= rpm - 250)
      .andThen(new RunCommand(()->index.setMotorSpeed(0.5)).withTimeout(0.25))
      .andThen(new InstantCommand(()->index.setMotorSpeed(0)))
      .andThen(new InstantCommand(()->intake.setHasNote(true)))
      );
    }

    public Command autoAimShot(double delay){
      return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new CMD_AutoAimOnDist(pivot, limelight, drivetrain).withTimeout(2.0),
            new RunCommand(()->shooter.shootFlywheelOnRPM(4500), shooter)
          ).andThen(
            new ParallelCommandGroup(
              new RunCommand(()->shooter.shootFlywheelOnRPM(4500), shooter),
              new RunCommand(()->index.setMotorSpeed(0.5), index)
            ).withTimeout(0.5)
          ).andThen(
            new ParallelCommandGroup(
              new InstantCommand(()->intake.setHasNote(true)),
              new InstantCommand(()->index.setMotorSpeed(0.0), index)
            )
          )
      );
    }

    public Command pathIntake(String path){
      return new ParallelCommandGroup(
        runIntake(),
        PathPlannerBase.followTrajectory(path)
      );
    }

    public Command noteDetectPath(String path){
      return new SequentialCommandGroup(
        PathPlannerBase.followTrajectory(path)
        .onlyWhile(() -> !NoteVision.getInstance().hasTarget())
          .andThen(
            runIntake()
            .deadlineWith(
                NoteVision.getInstance().autoIntake(() -> 2, SUB_Drivetrain.getInstance(), SUB_Intake.getInstance())
            ) 
      ));
    } 


    public Command runShooter(int rpm){
      return new RunCommand(
       ()->shooter.shootFlywheelOnRPM(rpm), shooter);
    }

    //TODO: Test CMD
    public Command runIntake(){
      return new ParallelCommandGroup(
        new InstantCommand(()->pivot.goToAngle(75)),
        new InstantCommand(()->index.starttimer()),
        new RunCommand(()->index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
        new RunCommand(()->intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
          ()->index.CurrentLimitSpike()).andThen(
            new InstantCommand(()->intake.setHasNote(true)),
        new RunCommand(()->index.setMotorSpeed(0.1)).withTimeout(0.025).andThen(new ParallelCommandGroup(
          new InstantCommand(()->index.setMotorSpeed(0)),
          new InstantCommand(()->intake.setMotorSpeed(0))))
      );
    }

    public Command stopIntake(){
      return new ParallelCommandGroup(
        new InstantCommand(()->index.setMotorSpeed(0)),
        new InstantCommand(()->intake.setMotorSpeed(0))
      );
    }

    public Command stopAll(){
      return new ParallelCommandGroup(
        new InstantCommand(()->index.setMotorSpeed(0)),
        new InstantCommand(()->intake.setMotorSpeed(0)),
        new InstantCommand(()->shooter.setMotorSpeed(1500))
      );
    }

    public Command setPivotSetpoint(double setpoint){
      return new InstantCommand(()->pivot.setPivotSetpoint(setpoint));
    }

    public Command dumpAuto(){
      return new ParallelCommandGroup(
        setPivotSetpoint(Constants.Pivot.kSpeakerAngleSP), 
        new CMD_Shoot(shooter, index, drivetrain, intake, pivot)
        );
    }

  // public Command shootWhenReady(){
  //   return shooter.runOnce(()->shooter.setRPM(4000)).until(()->(shooter.getFlywheelRPM() >= 3500)).andThen(index.runOnce(()->index.setMotorSpeed(.5)));
  // }

  

  

  // ====================================================================
  //                          Helpers
  // ====================================================================

  public void registerAllCommands(){
    NamedCommands.registerCommand("RunIntake", runIntake());
    NamedCommands.registerCommand("ScoringSequence", new CMD_Shoot(shooter, index, drivetrain, intake, pivot));
    NamedCommands.registerCommand("StopIntake", stopIntake());
    NamedCommands.registerCommand("StopAll", stopAll());
    NamedCommands.registerCommand("Amp Setpoint", setPivotSetpoint(Constants.Pivot.kSpeakerAngleSP+5));
    NamedCommands.registerCommand("Launch Setpoint", setPivotSetpoint(Constants.Pivot.kLowMidAngleSP));
    NamedCommands.registerCommand("LowShot Setpoint", setPivotSetpoint(Constants.Pivot.kLowAngleSP-10));
    NamedCommands.registerCommand("DumpAuto", new CMD_ShootSEQ(shooter, index, pivot));
  }

}
