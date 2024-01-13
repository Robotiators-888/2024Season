package frc.robot.utils;

import java.nio.file.Path;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.RobotManager;
import frc.robot.subsystems.SUB_Drivetrain;

/** This utility class is built for selecting made autos */
public class AutoGenerator {
  SUB_Drivetrain drivetrain;
  //RobotManager manager = RobotManager.getInstance();
  private final ArrayList<String> commandsList = new ArrayList<>();
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  
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
    /* chooser.addOption("Dummy 1", dummyPathOne());
    chooser.addOption("Dummy Donut", dummyPathDonut());
    chooser.addOption("8's HEHEHEHE", figureEight());
    */

    chooser.setDefaultOption("Score One", null);
    SmartDashboard.putData("Auto Selector", chooser);
  }

  // ====================================================================
  //                          Paths
  // ====================================================================
    String driveBack_Left = Filesystem.getDeployDirectory().toPath().resolve("Back_to_Podium").toString();


  // ====================================================================
  //                          Routines
  // ====================================================================
    public Command scoringSequence(){
      
      return new SequentialCommandGroup(
        
      );
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

  }

}
