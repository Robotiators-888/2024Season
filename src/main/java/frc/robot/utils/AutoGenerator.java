package frc.robot.utils;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.RobotManager;
import frc.robot.subsystems.SUB_Drivetrain;

/** This utility class is built for selecting made autos */
public class AutoGenerator {
  private SUB_Drivetrain drivetrain;
  private SendableChooser<Command> chooser = new SendableChooser<>();
  
  public AutoGenerator(SUB_Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    AutoBuilder.configureHolonomic(drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds, drivetrain::driveRobotRelative,
     new HolonomicPathFollowerConfig(4.5, 0.4, new ReplanningConfig())
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
  
  //TODO: Add Seq once branch is merges
    public Command scoringSequence(){
      return new SequentialCommandGroup(
        
      );
    }

    //TODO: Add intake instant when branch is merged
    public Command runIntake(double Speed){
      return new InstantCommand();
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
    NamedCommands.registerCommand("RunIntake", runIntake(0.75));
    NamedCommands.registerCommand("ScoringSequence", scoringSequence());
    NamedCommands.registerCommand("StopIntake", runIntake(0.0));
  }

}
