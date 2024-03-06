// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Pivot;
import frc.robot.utils.AutoGenerator;

/** Add your docs here. */
public class AUTO_JustShoot extends AutoPaths {
    

    @Override
    public Command load(AutoGenerator autos) {
        return Commands.sequence(
            autos.scoringSequence(Pivot.kSpeakerAngleSP, 2500)
        );

    }
}
