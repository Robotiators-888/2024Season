// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MidLine;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoPaths;
import frc.robot.utils.AutoGenerator;

/** Add your docs here. */
public class AUTO_MidLine_ThreeP extends AutoPaths {

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "TopStart_to_TopGP";
        String p2Name = "";
        String p3Name = "";
        String p4Name = "";
        String p5Name = "";
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
                
        );
    }
    
}
