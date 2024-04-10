// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.PathPlannerBase;

/** Add your docs here. */
public class AUTO_5P_Mid_NO_STAGE extends AutoPaths {

    @Override
    public Command load(AutoGenerator autos) {
        String p1Name = "4P_Middle_Extended";

        // Path misses first pickup
        return Commands.sequence(
                new AUTO_4P_Mid().load(autos),

                autos.pathIntake(p1Name));
    }

}
