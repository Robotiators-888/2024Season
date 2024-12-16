// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.utils.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
    // public static SUB_Limelight limelight = SUB_Limelight.getInstance();

    public static CommandXboxController Driver1 = new CommandXboxController(OIConstants.kDriver1ontrollerPort);
    public static CommandXboxController Driver2 = new CommandXboxController(OIConstants.kDriver2ControllerPort);

    public static SendableChooser<Boolean> standardPosChecker = new SendableChooser<>();

    public static SendableChooser<Double> delayChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        standardPosChecker.addOption("Odometery Init", Boolean.TRUE);
        standardPosChecker.setDefaultOption("ATag Init", Boolean.FALSE);
        SmartDashboard.putData("Standard Pose Chooser", standardPosChecker);


        delayChooser.setDefaultOption("0 Sec", 0.0);
        delayChooser.addOption("1 Sec", 1.0);
        delayChooser.addOption("2 Sec", 2.0);
        delayChooser.addOption("3 Sec", 3.0);
        delayChooser.addOption("4 Sec", 4.0);
        delayChooser.addOption("5 Sec", 5.0);
        SmartDashboard.putData("Delay Chooser", delayChooser);


        // Configure AutoBuilder last
        AutoBuilder
        .configureHolonomic(drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new HolonomicPathFollowerConfig(new PIDConstants(1.5, 0.0, 0.0), new PIDConstants(5.0, 0, 0),
                Constants.Drivetrain.kMaxModuleSpeed, Constants.Drivetrain.kTrackRadius, new ReplanningConfig()),
            AllianceFlipUtil::shouldFlip, drivetrain);


        // Configure the trigger bindings
        configureBindings();

        drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> drivetrain.drive(
                                -MathUtil.applyDeadband(Driver1.getRawAxis(1),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(Driver1.getRawAxis(0),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(Driver1.getRawAxis(4), OIConstants.kDriveDeadband),
                                true, true),
                        drivetrain));

        

        // pivot.set



        // Zero Heading
        Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

       

      
        // Robot relative drive
        Driver1.povDown().whileTrue(
                new RunCommand(
                        () -> drivetrain.drive(
                                -MathUtil.applyDeadband(
                                        Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                        Math.copySign(Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(Driver1.getRawAxis(4), OIConstants.kDriveDeadband),
                                false, true),
                        drivetrain));

                                        
      
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
    }

    
    public void robotPeriodic() {

    }
    public Command getAutonomousCommand() {
        try{
                // Load the path you want to follow using its name in the GUI
                PathPlannerPath path = PathPlannerPath.fromPathFile("GERSTNER_AUTO");

                // Create a path following command using AutoBuilder. This will also trigger event markers.
                return AutoBuilder.followPath(path);
        } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
        }
    }

public void teleopPeriodic() {
        // TODO Auto-generated method stub
}
    
}
