// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_5P_Mid_STAGE extends AutoPaths {

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "4P_Middle_Extended_STAGE";
        String p2Name = "5P_C3_Pickup";

        // Path misses first pickup
        return Commands.sequence(
                new AUTO_4P_Mid().load(autos),

                autos.setPivotSetpoint(Pivot.kLowAngleSP),
                PathPlannerBase.followTrajectory(p1Name),

               autos.visionAlignToNote());
                // autos.pathIntake(p2Name));
    }

}
