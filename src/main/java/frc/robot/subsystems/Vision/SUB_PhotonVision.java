// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

public class SUB_PhotonVision extends SubsystemBase {
  public static SUB_PhotonVision INSTANCE = null;

  private PhotonCamera cam = new PhotonCamera(PhotonVision.kCamName);
  private PhotonTrackedTarget bestTarget;
  public PhotonPoseEstimator poseEstimator;
  Transform3d robotToCam = new Transform3d(Units.inchesToMeters(-(-15.5 + 2.25)), Units.inchesToMeters(-(12.0 - 3.75)),
      Units.inchesToMeters(17.0), new Rotation3d(0, Units.degreesToRadians(-14), 0));

  private PhotonCamera noteCam = new PhotonCamera("NoteDetect");
  private PhotonTrackedTarget note;
  public boolean hasResults = false;

  public AprilTagFieldLayout at_field;

  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }

    return INSTANCE;
  }

  /** Creates a new SUB_PhotonVision. */
  private SUB_PhotonVision() {
    try {
      at_field = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2024_at_field.json"));
      SmartDashboard.putBoolean("FILE FOUND?", true);
    } catch (IOException e) {
      SmartDashboard.putBoolean("FILE FOUND?", false);
    }

    poseEstimator = new PhotonPoseEstimator(at_field,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }

  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public int getId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  public PhotonTrackedTarget getBestNote() {
    return note;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = cam.getLatestResult();

    if (result.hasTargets()) {
      bestTarget = result.getBestTarget();
    }

    // COMMENT OUT BELOW IF IT DOESN'T WORK
    // DO NOT COMMENT OUT ABOVE
    var intakeResults = noteCam.getLatestResult();

    if (intakeResults.hasTargets()) {
      hasResults = true;
      note = intakeResults.getBestTarget();
    } else {
      hasResults = false;
    }
  }
}
