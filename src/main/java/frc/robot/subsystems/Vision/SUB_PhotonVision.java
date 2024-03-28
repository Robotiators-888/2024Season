// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

public class SUB_PhotonVision extends SubsystemBase {
  public static SUB_PhotonVision INSTANCE = null;

  private PhotonCamera cam = new PhotonCamera(PhotonVision.kCamName);
  private PhotonTrackedTarget bestTarget;
  public PhotonPoseEstimator poseEstimator;
  AprilTagFieldLayout at_field;

  public class PosePair{
    public Pose2d pose;
    public double time;
    public Object pose2d;
    public PosePair(Pose2d pose, double time){
      this.pose = pose;
      this.time = time;
    }

  }

  public static SUB_PhotonVision getInstance(){
    if (INSTANCE == null){
      INSTANCE = new SUB_PhotonVision();
    }

    return INSTANCE;
  }

  /** Creates a new SUB_PhotonVision. */
  private SUB_PhotonVision() { 
    try {
    at_field = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2024_at_field.json"));
      SmartDashboard.putBoolean("FILE FOUND?", true);   
    } catch (IOException e){
      SmartDashboard.putBoolean("FILE FOUND?", false);      
    }      
  }
  
  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public PosePair getPose2dPhotonvision(){
    var res = cam.getLatestResult();
    if (res.hasTargets()) {
        double imageCaptureTime = res.getTimestampSeconds();
        var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
        int id = res.getBestTarget().getFiducialId();
        var camPose = at_field.getTagPose(id).get().transformBy(camToTargetTrans.inverse());
        return new PosePair(camPose.transformBy(PhotonVision.kCameraToRobot).toPose2d(), imageCaptureTime);

    }
    return null;
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
