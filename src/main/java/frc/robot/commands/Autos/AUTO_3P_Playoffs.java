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
public class AUTO_3P_Playoffs extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "TopStart_to_TopGP";
        String p2Name = "TopGP_to_1stGP";
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
        return Commands.sequence(
            autos.scoringSequence(Pivot.kSpeakerAngleSP-6,4000, 0.45),
            autos.resetOdometry(startingPose),

            autos.pathIntake(p1Name).withTimeout(4),
            autos.scoringSequence(Pivot.kLowAngleSP+2, 4000, 0.45),

            autos.pathIntake(p2Name).withTimeout(4),
            PathPlannerBase.followTrajectory(p3Name),
            autos.scoringSequence(Pivot.kLowAngleSP+3, 4500, 0.5)

           
            

        );
    }
    
}