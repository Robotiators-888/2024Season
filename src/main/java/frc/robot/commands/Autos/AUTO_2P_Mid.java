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
public class AUTO_2P_Mid extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "2P_Middle";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);

        // TODO Auto-generated method stub
        return Commands.sequence(
            autos.scoringSequence(Pivot.kSpeakerAngleSP, 2500),
            autos.resetOdometry(p1.getPreviewStartingHolonomicPose()),
            autos.pathIntake(p1Name),
            autos.scoringSequence(Pivot.kSpeakerAngleSP, 4000)
        );
    }
    
}