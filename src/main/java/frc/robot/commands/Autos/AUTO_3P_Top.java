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
public class AUTO_3P_Top extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "Top_to_TopGP";
        String p2Name = "TopGP_to_MiddleGP";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);
        PathPlannerPath p2 = PathPlannerPath.fromPathFile(p2Name);

        // TODO Auto-generated method stub
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
            autos.scoringSequence()
        );
    }
    
}
