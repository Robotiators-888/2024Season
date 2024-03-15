// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;

/** Add your docs here. */
public class AUTO_3P_Top extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "TopStart_to_TopGP";
        String p2Name = "TopGP_to_MiddleGP_Scorepos";
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
            autos.scoringSequence(Pivot.kSpeakerAngleSP, 2500),
            autos.resetOdometry(startingPose),

            autos.pathIntake(p1Name).withTimeout(4),
            autos.scoringSequence(Pivot.kLowAngleSP, 4000),

            autos.pathIntake(p2Name),
            new WaitCommand(.25),
            autos.scoringSequence(Pivot.kLowAngleSP, 4000)
        );
    }
    
}
