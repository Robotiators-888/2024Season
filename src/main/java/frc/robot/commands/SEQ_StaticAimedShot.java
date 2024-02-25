// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Shooter;

public class SEQ_StaticAimedShot extends SequentialCommandGroup {
  /** Creates a new PAR_AimedShot. */
  public SEQ_StaticAimedShot(
    SUB_Drivetrain drivetrain, 
    SUB_Shooter shooter, 
    SUB_Intake intake, 
    SUB_Index index, 
    SUB_Pivot pivot,
    SUB_Limelight limelight,
    CommandXboxController controller) {

    addCommands(
      new ParallelCommandGroup(
        //new CMD_AbsoluteDriveToTarget(drivetrain, Optional.of(goalPose)),
        new CMD_AimOnDist(pivot, limelight, drivetrain, controller)));
      // ),
      // new ParallelCommandGroup(
      //   new RunCommand(()->shooter.shootFlywheelOnRPM(4000), shooter),
      //   new SequentialCommandGroup(
      //     new WaitUntilCommand(()->shooter.getFlywheelRPM() == 4000),
      //     new RunCommand(()->index.setMotorSpeed(0.5), shooter)
      //   )
      // ),
      // new InstantCommand(()->shooter.setMotorSpeed(0)),
      // new InstantCommand(()->index.setMotorSpeed(0))
    //);
    addRequirements(drivetrain, shooter, pivot);
  }
}
