// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MidLine;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Pivot;
import frc.robot.commands.Autos.AutoPaths;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_MidLine_Bottom_Two extends AutoPaths {

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "BottomStart_5GP";
        String p2Name = "5th_to_BottomGP";
        
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
            autos.aimAtPoint(startingPose.getTranslation(), SUB_Drivetrain.getInstance()),
            autos.scoringSequence(Pivot.kSpeakerAngleSP-8,4000, 0.45),
            //autos.resetOdometry(startingPose, new Rotation2d(120)),
            autos.resetOdometry(startingPose),

            autos.pathIntake(p1Name).withTimeout(4),
            PathPlannerBase.followTrajectory(p2Name).withTimeout(4),
            autos.scoringSequence(Pivot.kLowAngleSP-1, 4500, 0.5)
        );
    }
    
}
