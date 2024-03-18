// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_2P_Mid extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "2P_Middle";
        String p2name = "Middle_to_MiddleHome";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);
        var alliance = DriverStation.getAlliance();
    
        Pose2d startingPose = null;
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red){
                startingPose = p1.flipPath().getPreviewStartingHolonomicPose();
            } else {
                startingPose = p1.getPreviewStartingHolonomicPose();
            }
        } 
        // TODO Auto-generated method stub
        return Commands.sequence(
            autos.scoringSequence(Pivot.kSpeakerAngleSP-5,4000, 0.45),
            autos.resetOdometry(startingPose),

            autos.pathIntake(p1Name).withTimeout(4),
            autos.scoringSequence(Pivot.kLowAngleSP+6, 4000, 0.33),

            PathPlannerBase.followTrajectory(p2name)
        );
    }
    
}
