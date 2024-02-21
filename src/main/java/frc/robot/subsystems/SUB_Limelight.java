
package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.Limelight.*;
import edu.wpi.first.wpilibj.DriverStation;

public class SUB_Limelight extends SubsystemBase {

  private DriverStation.Alliance allianceColor;
  public enum LED_Mode {
    FORCE_OFF,
    FORCE_ON,
    FORCE_BLINK
  }

  /** Creates a new SUB_Limelight. */
  public SUB_Limelight() {
    allianceColor = DriverStation.getAlliance().get();
    SmartDashboard.putString("ALLIANCE", allianceColor.toString());
  }

  /**
   * Returns robot pose in terms of x,y,z position on the field in meters, and roll, pitch yaw, in degrees. 
   * @implNote (X, Y, Z (meters), ROLL, PITCH, YAW (degrees))
   * @return Botpose network tables entry in Pose2d
   */
  public Pose2d getPose(){
    LimelightResults llresults = LimelightHelpers.getLatestResults(LIMELIGHT_NAME);

    // TODO: Limelight shits 
    switch (allianceColor){
      case Blue: return llresults.targetingResults.getBotPose2d_wpiRed();
      case Red: return llresults.targetingResults.getBotPose2d_wpiBlue();
      default: return null; // Default
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ID", LimelightHelpers.getFiducialID(LIMELIGHT_NAME));
  }

  public boolean getTv() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  /**
   * Crosshair offset to target x-value
   * 
   * @return double of offset of target x-value
   */
  public double getTx() {
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

  /**
   * Crosshair offset to target y-value
   * 
   * @return double of offset of target y-value
   */
  public double getTy() {
    return LimelightHelpers.getTY(LIMELIGHT_NAME);
  }

  /**
   * Returns the transform required to get to the target from the robot.
   * @return The Transform2D that represents the translation and rotation needed to get from robot to target.
   */
  public Pose3d getTargetTransform(){
    SmartDashboard.putNumber("X VAL TARGET", LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME).toPose2d().getX());
    SmartDashboard.putNumber("Y VAL TARGET", LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME).toPose2d().getY());
    return LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME);
  }
 

  /**
   * Sets LED Mode
   * @param state The enum value for the given state, either FORCE_OFF, FORCE_ON, or FORCE_BLINK
   */
  public void setLED(LED_Mode state) {
    switch (state){
      case FORCE_OFF: LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME); break;
      case FORCE_ON: LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_NAME); break;
      case FORCE_BLINK: LimelightHelpers.setLEDMode_ForceBlink(LIMELIGHT_NAME); break;
    }
  }

  /**
   * Sets pipeline
   * @param value The given index of the pipeline
   */
  public void setPipeline(int value) {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, value);
  }

  public double getCaptureLatency(){
    return LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME);
  }

  public double getPipelineLatency(){
    return LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME);
  }

  public double getTargetPitch(){
    return LimelightHelpers.getTY(LIMELIGHT_NAME);
  }
 
  public double getTargetYaw(){
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

}
