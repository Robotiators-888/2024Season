package frc.robot.subsystems.Vision.NoteDetect;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.PhotonVision.Camera;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.Vision.NoteDetect.NoteVision.TrackedNote;
import frc.robot.utils.LoggedTunableNumber;

public class NoteVisionIOPhotonVision implements NoteVisionIO {
    private final PhotonCamera cam;
    private final Camera camMeta;
    
    private static final LoggedTunableNumber targetPitchThreshold = new LoggedTunableNumber("Vision/Pitch Threshold", 0.0);

    public NoteVisionIOPhotonVision(Camera camera) {
        this.cam = new PhotonCamera(camera.hardwareName);
        this.camMeta = camera;

    }

    @Override
    public void updateInputs(NoteVisionIOInputs inputs) {
        inputs.connected = cam.isConnected();
        inputs.trackedNotes = new TrackedNote[0];
        if(!inputs.connected) return;
        inputs.trackedNotes = 
            cam
            .getLatestResult()
            .getTargets()
            .stream()
            // .filter((target) -> (camMeta.getRobotToCam().getRotation().getY() + target.getPitch()) < targetPitchThreshold.get())
            .map(this::resultToTargets)
            .toArray(TrackedNote[]::new);
    }

    private TrackedNote resultToTargets(PhotonTrackedTarget target) {
        var targetCamViewTransform = camMeta.getRobotToCam().plus(
            new Transform3d(
                new Translation3d(),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-target.getPitch()),
                    Units.degreesToRadians(-target.getYaw())
                )
            )
        );
        var distOut = targetCamViewTransform.getTranslation().getZ() / Math.tan(targetCamViewTransform.getRotation().getY());
        var distOff = distOut * Math.tan(targetCamViewTransform.getRotation().getZ());
        var camToTargetTranslation = new Translation3d(distOut, distOff, -camMeta.getRobotToCam().getZ());
        var fieldPos = new Pose3d(SUB_Drivetrain.getInstance().getPose())
            .transformBy(camMeta.getRobotToCam())
            .transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()))
            .toPose2d()
            .getTranslation();

        var confidence = Math.sqrt(target.getArea()) * NoteVision.confidencePerAreaPercent.get();

        return new TrackedNote(fieldPos, confidence);
    }
}
