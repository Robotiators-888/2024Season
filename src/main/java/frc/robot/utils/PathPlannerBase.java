package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUB_Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

/** Util class for all Path Planner auto builders/generators */
public class PathPlannerBase {

  static final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
  static final PathConstraints constraints = new PathConstraints(2, 2, 2, 2);

  public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return AutoBuilder.followPath(path);
    //drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds, drivetrain::driveRobotRelative,
    // return new FollowPathHolonomic(path, drivetrain::getPose, drivetrain::getChassisSpeeds, drivetrain::driveRobotRelative, 
    //   new HolonomicPathFollowerConfig(Constants.Drivetrain.kMaxModuleSpeed, Constants.Drivetrain.kTrackRadius, new ReplanningConfig())
    //   , ()->{
    //     var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                     return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //   }, drivetrain);
  }

  public static Command followTrajectory(String PathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
    return AutoBuilder.followPath(path);
  }

  




}
