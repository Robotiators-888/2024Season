// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_3P_Mid extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "2P_Middle";
        String p2Name = "3P_Middle_to_BottomStraight";
        String p3Name = "3P_BottomGP_Out_Score";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);
        PathPlannerPath p2 = PathPlannerPath.fromPathFile(p2Name);
        PathPlannerPath p3 = PathPlannerPath.fromPathFile(p3Name);

        return Commands.sequence(
            autos.setPivotSetpoint(Pivot.kAmpAngleSP),
            autos.scoringSequence(),
            autos.resetOdometry(p1.getPreviewStartingHolonomicPose()),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p1Name)
            ),
            autos.setPivotSetpoint(55),
            new WaitCommand(.75),
            autos.scoringSequence(),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p2Name)
            ),
            PathPlannerBase.followTrajectory(p3Name),
            autos.setPivotSetpoint(55),
            new WaitCommand(.25),
            autos.scoringSequence()

        );
    }
    
}
