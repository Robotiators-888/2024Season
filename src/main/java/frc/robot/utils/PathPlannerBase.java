// package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUB_Drivetrain;
import java.util.HashMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

/** Util class for all Path Planner auto builders/generators */
public class PathPlannerBase {

  static final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
  static final PathConstraints constraints = new PathConstraints(2, 2, 2, 2);

  public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(path, ()->drivetrain.getPose(), ()->drivetrain.getChassisSpeeds(), drivetrain::driveRobotRelative, 
      new HolonomicPathFollowerConfig(4.5,
      0,
        null)
      , null, null);
  }
}
