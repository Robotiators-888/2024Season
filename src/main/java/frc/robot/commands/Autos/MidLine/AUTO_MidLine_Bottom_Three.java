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
public class AUTO_MidLine_Bottom_Three extends AutoPaths {

    @Override
    public Command load(AutoGenerator autos) {
        String p0Name = "ROT_To_152Deg";
        String p1Name = "BottomStart_5GP";
        String p2Name = "5th_to_BottomGP";
        String p3Name = "Bottom_to_4GP";
        String p4Name = "4th_to_BottomGP";
        
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p0Name);
        var alliance = DriverStation.getAlliance();
        
        // 3.07, 3.09, 147.99

        Pose2d startingPose = null;
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red){
                startingPose = p1.flipPath().getPreviewStartingHolonomicPose();
            } else {
                startingPose = p1.getPreviewStartingHolonomicPose();
            }
        }

        return new SequentialCommandGroup(
            autos.resetOdometry(startingPose), 
            PathPlannerBase.followTrajectory(p0Name).withTimeout(1),
            autos.scoringSequence(Pivot.kSpeakerAngleSP-6,4000, 0.45),
            

            autos.pathIntake(p1Name).withTimeout(4),
            PathPlannerBase.followTrajectory(p2Name).withTimeout(4),
            autos.scoringSequence(Pivot.kLowAngleSP-1, 4500, 0.5),
           // autos.autoAimShot(0.0),

            autos.pathIntake(p3Name).withTimeout(4),
            PathPlannerBase.followTrajectory(p4Name).withTimeout(4),
            autos.scoringSequence(Pivot.kLowAngleSP-1, 4500, 0.75)

            //autos.autoAimShot(0.0)

        );
    }
    
}
