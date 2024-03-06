// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Silly;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Pivot;
import frc.robot.commands.Autos.AutoPaths;
import frc.robot.utils.AutoGenerator;

/** Add your docs here. */
public class AUTO_Silly_BelineFirst extends AutoPaths{

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "SillyAuto_Start_Top_1";
        String p2Name = "SillyAuto_GP_to_Mid";
        PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);

        // TODO Auto-generated method stub
        return new SequentialCommandGroup(
            autos.scoringSequence(Pivot.kAmpAngleSP, 2500),
            autos.resetOdometry(p1.getPreviewStartingHolonomicPose()),
            autos.pathIntake(p1Name),
            autos.pathIntake(p2Name),
            autos.scoringSequence(Pivot.kLowAngleSP, 4000)
        );
    }
    
}
