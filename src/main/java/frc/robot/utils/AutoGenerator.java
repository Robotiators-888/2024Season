package frc.robot.utils;

import java.time.Instant;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
//import frc.robot.RobotManager;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;

/** This utility class is built for selecting made autos */
public class AutoGenerator {
  private SUB_Drivetrain drivetrain;
  SUB_Index index;
  SUB_Intake intake;
  SUB_Shooter shooter; 
  SUB_Pivot pivot;
  SUB_Limelight limelight;

  private SendableChooser<Command> chooser = new SendableChooser<>();
  
  public AutoGenerator(SUB_Drivetrain drivetrain, SUB_Index index, SUB_Intake intake, SUB_Shooter shooter, SUB_Pivot pivot) {
    this.drivetrain = drivetrain;
    this.index = index;
    this.intake = intake;
    this.shooter = shooter;
    this.pivot = pivot;
    
    AutoBuilder.configureHolonomic(drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds, drivetrain::driveRobotRelative,
     new HolonomicPathFollowerConfig(Constants.Drivetrain.kMaxModuleSpeed, Constants.Drivetrain.kTrackRadius, new ReplanningConfig())
    , ()->{
      var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
    }, drivetrain);

    registerAllCommands();
   
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selector", chooser);
  }

  // ====================================================================
  //                          Paths
  // ====================================================================
    String driveBack_Left = Filesystem.getDeployDirectory().toPath().resolve("Back_to_Podium").toString();


  // ====================================================================
  //                          Routines
  // ====================================================================
  
  //TODO: Test SEQ
    public Command scoringSequence(){
      return new RunCommand(()->shooter.setRPM(4000))
      .withTimeout(0.75)
      .andThen(new RunCommand(()->index.setMotorSpeed(0.75)).withTimeout(0.5).andThen(new InstantCommand(()->stopAll())));
    }

    //TODO: Test CMD
    public Command runIntake(){
      return new ParallelCommandGroup(
        new InstantCommand(()->pivot.goToAngle(75)),
        new RunCommand(()->index.setMotorSpeed(Constants.Intake.kIndexSpeed)),
        new RunCommand(()->intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(()->index.getTopBannerSensor()).andThen(
        new RunCommand(()->index.setMotorSpeed(0.1)).withTimeout(0.025).finallyDo(()->stopAll())
      ).finallyDo(()->stopAll());
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
        setPivotSetpoint(Constants.Pivot.kAmpAngleSP), 
        scoringSequence()
        ).finallyDo(()->stopAll());
    }

  /**
   * @return Returns chosen auto on Smartdashboard
   */
    public Command getSelectedAuto() {
      return chooser.getSelected();
    }

  // ====================================================================
  //                          Helpers
  // ====================================================================

  public void registerAllCommands(){
    NamedCommands.registerCommand("RunIntake", runIntake());
    NamedCommands.registerCommand("ScoringSequence", scoringSequence());
    NamedCommands.registerCommand("StopIntake", stopIntake());
    NamedCommands.registerCommand("StopAll", stopAll());
    NamedCommands.registerCommand("Amp Setpoint", setPivotSetpoint(Constants.Pivot.kAmpAngleSP));
    NamedCommands.registerCommand("Launch Setpoint", setPivotSetpoint(Constants.Pivot.kLowMidAngleSP));
    NamedCommands.registerCommand("LowShot Setpoint", setPivotSetpoint(Constants.Pivot.kLowAngleSP));
    NamedCommands.registerCommand("DumpAuto", dumpAuto());
  }

}
