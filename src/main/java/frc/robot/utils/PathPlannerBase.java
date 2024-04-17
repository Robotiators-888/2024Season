package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUB_Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/** Util class for all Path Planner auto builders/generators */
public class PathPlannerBase {

  static final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
  static final PathConstraints constraints = new PathConstraints(3.75, 1.0, 1.0, 0.75);

  public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);


    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public static Command followTrajectory(String PathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  




}
