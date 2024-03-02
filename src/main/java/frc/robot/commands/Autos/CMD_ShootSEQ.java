// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_ShootSEQ extends SequentialCommandGroup {


  SUB_Shooter shooter;
  SUB_Index index;
  SUB_Pivot pivot;
  CMD_Shoot CMD_shoot;
    /** Creates a new CMD_ShootSEQ. */
  public CMD_ShootSEQ(SUB_Shooter shooter, SUB_Index index, SUB_Pivot pivot) {

    this.shooter = shooter;
    this.index = index;
    this.pivot = pivot;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new InstantCommand(()->pivot.setPivotSetpoint(Pivot.kAmpAngleSP)), new CMD_Shoot(shooter, index));
  }

 

}
