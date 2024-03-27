// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Limelight;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Autos.MidLine.*;
import frc.robot.commands.Autos.Silly.AUTO_Silly_GERSTNER;
import frc.robot.commands.Autos.Troubles.*;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;

/** Add your docs here. */
public class AutoSelector {
    private SendableChooser<AutoPaths> chooser = new SendableChooser<AutoPaths>();

    AutoGenerator autos;

    public AutoSelector(SUB_Drivetrain drivetrain, SUB_Index index, SUB_Intake intake, SUB_Shooter shooter, SUB_Pivot pivot, SUB_Limelight limelight, CommandXboxController driver1){
        autos = new AutoGenerator(drivetrain, index, intake, shooter, pivot, limelight, driver1);

        chooser.setDefaultOption("1P_Stay", new AUTO_JustShoot());
        chooser.addOption("2P_Top", new AUTO_2P_Top());
        chooser.addOption("2P_Middle", new AUTO_2P_Mid());

        chooser.addOption("2P_Midline_Bottom_Two", new AUTO_MidLine_Bottom_Two());

        chooser.addOption("3P_Top", new AUTO_3P_Top());
        chooser.addOption("3P_Middle", new AUTO_3P_Mid());
        chooser.addOption("3P_Midline_ONLY_Bottom", new AUTO_MidLine_Bottom_Three());
        chooser.addOption("3P_Midline_Top", new AUTO_3P_Playoffs());


        // chooser.addOption("3P_Bottom", new AUTO_3P_Bottom());
        chooser.addOption("4P_MidLine_3_1st", new AUTO_MidLine_FourP_One());
        //chooser.addOption("3P_MidLine_3_2nd", new AUTO_MidLine_FourP_Two());

        chooser.addOption("4P_Top", new AUTO_4P_Top());
        //chooser.addOption("4P_Middle", new AUTO_4P_Mid());

        //chooser.addOption("Beline", new AUTO_MidLine_Beline());
        chooser.addOption("Gerstner Auto", new AUTO_Silly_GERSTNER());

        chooser.addOption("1 Meter back", new AUTO_1_Meter());
        chooser.addOption("2 Meter", new AUTO_2_Meter());
        chooser.addOption("5 Meter", new AUTO_5_Meter());


        SmartDashboard.putData("Auto Selector", chooser);
    }

    public Command getSelected() {
        return chooser.getSelected().load(autos);
    }

}
