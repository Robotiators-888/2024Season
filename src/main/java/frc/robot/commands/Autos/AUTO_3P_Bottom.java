// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;

/** Add your docs here. */
public class AUTO_3P_Bottom extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "BottomStart_to_BottomGP";
        String p2Name = "BottomScore_to_MiddleGP";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);

        return Commands.sequence(
            autos.scoringSequence(Pivot.kSpeakerAngleSP,2500),

            autos.resetOdometry(p1.getPreviewStartingHolonomicPose()),
            autos.pathIntake(p1Name),
            autos.scoringSequence(Pivot.kLowAngleSP,4000),

            autos.pathIntake(p2Name),
            autos.scoringSequence(Pivot.kLowAngleSP,4000)
        );
    }
    
}