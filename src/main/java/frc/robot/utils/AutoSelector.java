// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Autos.AutoPaths;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;

/** Add your docs here. */
public class AutoSelector {
    private SendableChooser<AutoPaths> chooser = new SendableChooser<AutoPaths>();

    AutoGenerator autos;

    public AutoSelector(SUB_Drivetrain drivetrain, SUB_Index index, SUB_Intake intake, SUB_Shooter shooter, SUB_Pivot pivot){
        autos = new AutoGenerator(drivetrain, index, intake, shooter, pivot);

        chooser.setDefaultOption("1P_Stay", new AUTO_JustShoot());
        chooser.addOption("2P_Middle", new AUTO_2P_Mid());
        chooser.addOption("2P_Top", new AUTO_2P_Top());
        chooser.addOption("3P_Top", new AUTO_3P_Top());
        chooser.addOption("4P_Top", new AUTO_4P_Top());


        SmartDashboard.putData("Auto Selector", chooser);
    }

    public Command getSelected() {
        return chooser.getSelected().load(autos);
    }

}
