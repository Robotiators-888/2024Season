// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_4P_Top extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "TopStart_to_TopGP";
        String p2Name = "TopGP_to_MiddleGP_Scorepos";
        String p3Name = "4P_MiddleScorePose_to_BottomGP+Score";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);

        return Commands.sequence(
            autos.setPivotSetpoint(Pivot.kAmpAngleSP),
            autos.scoringSequence(),
            autos.resetOdometry(p1.getPreviewStartingHolonomicPose()),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p1Name)
            ),
            autos.setPivotSetpoint(Pivot.kLowAngleSP),
            autos.scoringSequence(),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p2Name)
            ),
            autos.setPivotSetpoint(Pivot.kLowAngleSP),
            autos.scoringSequence(),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p3Name)
            ),
            autos.setPivotSetpoint(Pivot.kLowAngleSP),
            autos.scoringSequence()
        );
    }
    
}
