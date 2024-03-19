// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

public class SUB_PhotonVision extends SubsystemBase {
  /** Creates a new SUB_PhotonVision. */
  public SUB_PhotonVision() {}

private PhotonCamera cam = new PhotonCamera(PhotonVision.kCamName);



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
