// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MidLine;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Pivot;
import frc.robot.commands.Autos.AutoPaths;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_MidLine_Second extends AutoPaths {

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "Top_to_topGP";
        String p2Name = "TopGP_to_2ndGP";
        String p3Name = "1st2nd_ReturnToShoot";
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
    

        return new SequentialCommandGroup(
            autos.scoringSequence(Pivot.kSpeakerAngleSP, 2000),
            autos.resetOdometry(startingPose),
            
            autos.pathIntake(p1Name),

            autos.scoringSequence(Pivot.kLowAngleSP, 4000),

            autos.pathIntake(p2Name),

            PathPlannerBase.followTrajectory(p3Name),
            autos.scoringSequence(Pivot.kLowAngleSP, 4000)
        );
    }


    
}
