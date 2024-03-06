// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_3P_Top extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "TopStart_to_TopGP";
        String p2Name = "TopGP_to_MiddleGP_Scorepos";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);

        return Commands.sequence(
            autos.setPivotSetpoint(Pivot.kAmpAngleSP),
            autos.scoringSequenceClose(),
            autos.resetOdometry(p1.getPreviewStartingHolonomicPose()),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p1Name)
            ),
            autos.setPivotSetpoint(Pivot.kAutoShootingLowAngleSP),
            new WaitCommand(.25),
            autos.scoringSequence(),
            new ParallelCommandGroup(
                autos.runIntake(),
                PathPlannerBase.followTrajectory(p2Name)
            ),
            autos.setPivotSetpoint(Pivot.kAutoShootingLowAngleSP),
            new WaitCommand(.25),
            autos.scoringSequence()
        );
    }
    
}
