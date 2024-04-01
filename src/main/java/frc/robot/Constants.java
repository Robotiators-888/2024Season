// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;



import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.libs.PIDGains;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriver1ontrollerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kDriver2ControllerPort =1;
  }

  public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(648);
        public static final double fieldWidth =  Units.inchesToMeters(324);

        public static final Translation2d speakerAimPoint = new Translation2d(0.240581, 5.547755);

        public static final Pose2d subwooferFront =     new Pose2d(new Translation2d(1.45, 5.55), Rotation2d.fromDegrees(+180));
        public static final Pose2d subwooferAmp =       new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(-120));
        public static final Pose2d subwooferSource =    new Pose2d(new Translation2d(0.71, 4.57), Rotation2d.fromDegrees(+120));
        
        public static final Pose2d amp =                new Pose2d(new Translation2d(1.83, 7.61), Rotation2d.fromDegrees(-90));
        public static final Pose2d podium =             new Pose2d(new Translation2d(2.76, 4.44), Rotation2d.fromDegrees(+157.47));

        public static final Pose2d pathfindSpeaker =    new Pose2d(new Translation2d(3.45, 5.55), Rotation2d.fromDegrees(+180));
        public static final Pose2d pathfindSource =     new Pose2d(new Translation2d(13.41, 1.54), Rotation2d.fromDegrees(+180));

        public static final double podiumToSpeakerDist =    speakerAimPoint.getDistance(podium.getTranslation());
        public static final double subwooferToSpeakerDist = speakerAimPoint.getDistance(subwooferFront.getTranslation());
    }

  public static final class VortexMotorConstants{
    public static final double kFreeSpeedRpm = 6784;
  }

  public static class Swerve {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(2.85);
    // Thrifty tread 2.95in
    // Orange Tread 2.70
    // Black Rev 2.95
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians

    public static final double kTurningVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final double headingTolerance = Degrees.of(1).in(Radians);
    // Max Rot = Max Linear ((meters/sec)/60 (m/s)) / radius
    public static final double kMaxRotationalSpeed = (kDrivingMotorFreeSpeedRps/60) / Drivetrain.kTrackRadius;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    

  }

  public static final class Drivetrain {
    public static final int kFRONT_LEFT_DRIVE_MOTOR_CANID = 24;
    public static final int kFRONT_LEFT_STEER_MOTOR_CANID = 25;
    public static final int kFRONT_RIGHT_DRIVE_MOTOR_CANID = 20;
    public static final int kFRONT_RIGHT_STEER_MOTOR_CANID = 21;
    public static final int kBACK_LEFT_DRIVE_MOTOR_CANID = 26;
    public static final int kBACK_LEFT_STEER_MOTOR_CANID = 27;
    public static final int kBACK_RIGHT_DRIVE_MOTOR_CANID = 22;
    public static final int kBACK_RIGHT_STEER_MOTOR_CANID = 23;

//Original IDS
    // public static final int kFRONT_LEFT_DRIVE_MOTOR_CANID = 20;
    // public static final int kFRONT_LEFT_STEER_MOTOR_CANID = 21;
    // public static final int kFRONT_RIGHT_DRIVE_MOTOR_CANID = 22;
    // public static final int kFRONT_RIGHT_STEER_MOTOR_CANID = 23;
    // public static final int kBACK_LEFT_DRIVE_MOTOR_CANID = 24;
    // public static final int kBACK_LEFT_STEER_MOTOR_CANID = 25;
    // public static final int kBACK_RIGHT_DRIVE_MOTOR_CANID = 26;
    // public static final int kBACK_RIGHT_STEER_MOTOR_CANID = 27;

    public static final Rotation2d shooterSide = new Rotation2d(0);
    public static final Rotation2d intakeSide = new Rotation2d(180);

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.74;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 100000; // radians per second
    public static final double kMagnitudeSlewRate = 100000; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    //31inches by 24inches
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(31);

    public static final double kTrackRadius = Units.inchesToMeters(19.6 * Math.sqrt(2)/2);
    public static final double kMaxModuleSpeed = Units.feetToMeters(15);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2.0;
    public static final double kFrontRightChassisAngularOffset = 0.0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2.0;

    public static final boolean kGyroReversed = true;

    public static final double kGyroRotation = 0;
    }

    public static final class Intake{
      public static final int kINTAKE_MOTOR_CANID = 34;

      public static final double kIndexSpeed = 0.3;
      public static final double kOutakeSpeed = -0.5;
      public static final double kOutakeRPM = VortexMotorConstants.kFreeSpeedRpm;
      public static final double kIntakingSpeed = 0.35;
    }
    public static final class Climber{
      public static final double kUpSpeed = -1;
      public static final double kDownSpeed = 1;
    }
  
     public static final class Pivot {
      public static final int kPIVOT_ROTATE_MOTOR_CANID = 35;
      public static final double lbsArm = 45.0;
      public static final double kPivotManualDeadband = .05;
      public static final double kArmManualScale = .1;
      public static final int kCurrentLimit = 40; 
      //Setpoints:
      public static double khome = 0;

      public static final double kPivotOffset = 241.6031957;

      public static final double kAngularEncoderOffsetInDeg = 0;
      public static final double kMaxArmAngle = 106;
      public static final double kMinArmAngle = 49.8;
      //
      public static final double PID_kP = 0.11425;
      public static final int PID_kI = 0;
      public static final double PID_kD = 0.021;
      public static final double FF_kA = 0;
      public static final double FF_kG = 0; //amount of volts to Overcome gravity on the arm
      public static final double FF_kS = 0.1;
      public static final double FF_kV = 0.0204; 

      public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(30 , 30);
      public static final SimpleMotorFeedforward kArmFeedforward = new SimpleMotorFeedforward(FF_kS, FF_kV);
      public static final PIDGains kArmPositionGains = new PIDGains(0.0, 0.0, 0.0);
      //public static final double kGroundPosition = 0.1;
      public static final double kmaxVelocity = 6.47*Math.PI;
      public static final double kmaxAcceleration = 4.27;

      public static final double kHighAngleSP = 105.0;
      public static final double kSpeakerAngleSP = 86.0;
      public static final double kSideSP = 80;
      public static final double kLowMidAngleSP = 75.0;
      public static final double kLowAngleSP = 55;
      public static final double kSourceAngle = 88.0;



        
    }

  public static class PhotonVision{
    public static final double targetWidth = 0;
    public static final double targetHeight = 0;
    public static final String kCamName = "AprilTagCam";

    //Camera Specs
    public static final double camDiagFOV = 0; // degrees
    public static final double camPitch = 0; // degrees
    public static final double camHeightOffGround = 0; // meters
    public static final int camResolutionWidth = 0; // pixels
    public static final int camResolutionHeight = 0; // pixels
    public static final double minTargetArea = 0; // square pixels
    public static final Rotation3d cameraRotation = new Rotation3d(0,-14,0);
    public static final Transform3d kCameraToRobot = new Transform3d(Units.inchesToMeters(-15.5 + 2.25), Units.inchesToMeters(12.0 - 3.75), Units.inchesToMeters(15.5),cameraRotation);
  
    public static enum Camera {
            NoteVision(
                "NoteCamera",
                new Transform3d(
                    new Translation3d(
                        Inches.of(-Drivetrain.kWheelBase/2),
                        Inches.of(+3.5),
                        Inches.of(+Drivetrain.kTrackWidth/2)
                    ),
                    new Rotation3d(
                        0,
                        0,
                        Math.PI
                    )
                )
            ),

            AprilTagShooterSide(
              "AprilTagCam",
              new Transform3d(
                new Translation3d(
                  Inches.of(-15.5 + 2.25),
                  Inches.of(12.0 - 3.75),
                  Inches.of(15.5)
                ),
                new Rotation3d(
                  0,
                  -14,
                  0
                )
              )
            )
            ;
            public final String hardwareName;
            private final Transform3d intermediateToCamera;
            private Supplier<Transform3d> robotToIntermediate;
            Camera(String hardwareName, Transform3d finalToCamera) {
                this.hardwareName = hardwareName;
                this.intermediateToCamera = finalToCamera;
                this.robotToIntermediate = Transform3d::new;
            }
            @SuppressWarnings("unused")
            private static Transform3d robotToCameraFromCalibTag(Transform3d robotToCalibTag, Transform3d cameraToCalibTag) {
                return robotToCalibTag.plus(cameraToCalibTag.inverse());
            }
            public Camera withRobotToIntermediate(Supplier<Transform3d> robotToFinal) {
                this.robotToIntermediate = robotToFinal;
                return this;
            }

            public Transform3d getRobotToCam() {
                return robotToIntermediate.get().plus(intermediateToCamera);
            }
        }
  
  }



  public static class Limelight{
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double VERTICAL_FOV_DEGREES = 49.7;
    public static final double HEIGHT_TARGET_METERS = Units.inchesToMeters(53 + 7/8);
    public static final double HEIGHT_CAMERA_METERS = 0;
    public static final double CAMERA_ANGLE_DEGREES = 0; // might be 90 (+ = ___, - = ___)
  }
  
}

