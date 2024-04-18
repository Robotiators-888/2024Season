// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Autos.MidLine.*;
import frc.robot.commands.Autos.Silly.AUTO_Silly_GERSTNER;
import frc.robot.commands.Autos.Troubles.*;

/** Add your docs here. */
public class AutoSelector {
    private SendableChooser<AutoPaths> chooser = new SendableChooser<AutoPaths>();

    AutoGenerator autos;

    public AutoSelector(CommandXboxController driver1){
        autos = new AutoGenerator(driver1);

        chooser.setDefaultOption("1P_Stay", new AUTO_JustShoot());
        chooser.addOption("2P_Top", new AUTO_2P_Top());
        chooser.addOption("2P_Middle", new AUTO_2P_Mid());

        chooser.addOption("2P_Midline_Bottom_Two", new AUTO_MidLine_Bottom_Two());

        chooser.addOption("3P_Top", new AUTO_3P_Top());
        chooser.addOption("3P_Middle", new AUTO_3P_Mid());
        chooser.addOption("3P_Midline_ONLY_Bottom", new AUTO_MidLine_Bottom_Three());
        chooser.addOption("3P_Midline_Top", new AUTO_3P_Playoffs());


        chooser.addOption("4P_MidLine_3_1st", new AUTO_MidLine_FourP_One());
        // chooser.addOption("4P_Autoshot", new AUTO_MidLine_FourP_One_AIM());

        chooser.addOption("4P_Top", new AUTO_4P_Top());
        chooser.addOption("4P_Middle", new AUTO_4P_Mid());
        chooser.addOption("4P_Middle_REV", new AUTO_4P_Mid_REV());

        // DCMP AUTO
        // chooser.addOption("4P_Middle PLAYOFFS!!!!!", new AUTO_4P_Mid_PLAYOFFS());


        chooser.addOption("5P_Middle_No_Stage", new AUTO_5P_Mid_NO_STAGE());
        chooser.addOption("5P_Middle_STAGE", new AUTO_5P_Mid_STAGE());

        chooser.addOption("Gerstner Auto", new AUTO_Silly_GERSTNER());

        // chooser.addOption("1 Meter back", new AUTO_1_Meter());
        // chooser.addOption("2 Meter", new AUTO_2_Meter());
        // chooser.addOption("5 Meter", new AUTO_5_Meter());


        SmartDashboard.putData("Auto Selector", chooser);
    }

    public Command getSelected() {
        return chooser.getSelected().load(autos);
    }

}
