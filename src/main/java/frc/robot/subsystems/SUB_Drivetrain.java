// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.*;
import org.littletonrobotics.junction.Logger;

public class SUB_Drivetrain extends SubsystemBase {
  public final Field2d m_field = new Field2d();
  /** Creates a new Drivetrain. */
  
  private final MAXSwerveModule frontLeft 
    = new MAXSwerveModule(Constants.Drivetrain.kFRONT_LEFT_DRIVE_MOTOR_CANID, 
    Constants.Drivetrain.kFRONT_LEFT_STEER_MOTOR_CANID, Constants.Drivetrain.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight 
    = new MAXSwerveModule(Constants.Drivetrain.kFRONT_RIGHT_DRIVE_MOTOR_CANID, 
    Constants.Drivetrain.kFRONT_RIGHT_STEER_MOTOR_CANID, Constants.Drivetrain.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule backLeft 
    = new MAXSwerveModule(Constants.Drivetrain.kBACK_LEFT_DRIVE_MOTOR_CANID, 
    Constants.Drivetrain.kBACK_LEFT_STEER_MOTOR_CANID, Constants.Drivetrain.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule backRight 
    = new MAXSwerveModule(Constants.Drivetrain.kBACK_RIGHT_DRIVE_MOTOR_CANID, 
    Constants.Drivetrain.kBACK_RIGHT_STEER_MOTOR_CANID, Constants.Drivetrain.kBackRightChassisAngularOffset);    


  AHRS navx = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter =
      new SlewRateLimiter(Constants.Drivetrain.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter =
      new SlewRateLimiter(Constants.Drivetrain.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

 // Odometry class for tracking robot pose
 SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
  Constants.Drivetrain.kDriveKinematics,
  Rotation2d.fromDegrees(-navx.getAngle()),
  new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
  }, new Pose2d());

  public SUB_Drivetrain() {}

  @Override
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(-navx.getAngle()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      });
    m_field.setRobotPose(m_odometry.getEstimatedPosition());   
    // This method will be called once per scheduler run

    Logger.getInstance().recordOutput("Drivetrain/Robot Pose", m_odometry.getEstimatedPosition());

    Logger.getInstance().recordOutput("Driving Velocity", frontLeft.getVelocityDrive());
    Logger.getInstance().recordOutput("Steering Velocity", frontLeft.getVelocitySteer());
    Logger.getInstance().recordOutput("Driving Velocity", frontRight.getVelocityDrive());
    Logger.getInstance().recordOutput("Steering Velocity", frontRight.getVelocitySteer());
    Logger.getInstance().recordOutput("Driving Velocity", backLeft.getVelocityDrive());
    Logger.getInstance().recordOutput("Steering Velocity", backLeft.getVelocitySteer());
    Logger.getInstance().recordOutput("Driving Velocity", backRight.getVelocityDrive());
    Logger.getInstance().recordOutput("Steering Velocity", backRight.getVelocitySteer());

    // SmartDashboard.putNumber("Front Left Angle", frontLeft.);

    SmartDashboard.putNumber("rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumberArray(
        "Odometry",
        new double[] {getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(navx.getAngle()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate =
            Math.abs(Constants.Drivetrain.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.Drivetrain.kMaxAngularSpeed;

    var swerveModuleStates =
        Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(-navx.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate() * (Constants.Drivetrain.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Allows for vision measurements to be added to drive odometry.
   * @param visionPose The pose supplied by getPose() in SUB_Limelight
   */
  public void addVisionMeasurement(Pose2d visionPose){
    m_odometry.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
  }
}
