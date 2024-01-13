package frc.robot.utils;

import java.util.HashMap;

//import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
//import frc.robot.RobotManager;
import frc.robot.subsystems.SUB_Drivetrain;

/** This utility class is built for selecting made autos */
public class AutoGenerator {
  SUB_Drivetrain drivetrain;
  //RobotManager manager = RobotManager.getInstance();
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // ====================================================================
  //                          Trajectories
  // ====================================================================

  
  // ====================================================================
  //                          Routines
  // ====================================================================

  public AutoGenerator(SUB_Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    

    /* chooser.addOption("Dummy 1", dummyPathOne());
    chooser.addOption("Dummy Donut", dummyPathDonut());
    chooser.addOption("8's HEHEHEHE", figureEight());
    */

    chooser.setDefaultOption("Score One", null);
    SmartDashboard.putData("Auto Selector", chooser);
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


}
