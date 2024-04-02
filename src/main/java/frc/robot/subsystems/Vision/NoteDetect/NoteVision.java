package frc.robot.subsystems.Vision.NoteDetect;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.PhotonVision.Camera;
import frc.robot.subsystems.*;
import frc.robot.utils.LazyOptional;
import frc.robot.utils.LoggedTunableNumber;

public class NoteVision extends SubsystemBase {
    public static NoteVision INSTANCE = null;

    private final NoteVisionIO noteVisionIO;
    private final NoteVisionIOInputsAutoLogged inputs = new NoteVisionIOInputsAutoLogged();

    private final ArrayList<TrackedNote> noteMemories = new ArrayList<>();

    private static final LoggedTunableNumber updateDistanceThreshold = new LoggedTunableNumber("Vision/Note/updateDistanceThreshold", 5);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Note/posUpdatingFilteringFactor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Note/Confidence/UpdatingFilteringFactor", 0.5);
    public static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/Note/Confidence/PerAreaPercent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/Note/Confidence/DecayPerSecond", 3);
    private static final LoggedTunableNumber priorityPerConfidence = new LoggedTunableNumber("Vision/Note/Priority/PriorityPerConfidence", 4);
    private static final LoggedTunableNumber priorityPerDistance = new LoggedTunableNumber("Vision/Note/Priority/PriorityPerDistance", -2);
    private static final LoggedTunableNumber acquireConfidenceThreshold = new LoggedTunableNumber("Vision/Note/Target Threshold/Acquire", 0);
    private static final LoggedTunableNumber detargetConfidenceThreshold = new LoggedTunableNumber("Vision/Note/Target Threshold/Detarget", 0.5);

    private Optional<TrackedNote> optIntakeTarget = Optional.empty();
    private boolean intakeTargetLocked = false;

    public static NoteVision getInstance(){
        if(INSTANCE == null){
            INSTANCE = new NoteVision(new NoteVisionIOPhotonVision(Camera.NoteVision));
        }

        return INSTANCE;
    }

    private NoteVision(NoteVisionIO noteVisionIO) {
        System.out.println("[Init NoteVision] Instantiating NoteVision");
        this.noteVisionIO = noteVisionIO;
        System.out.println("[Init NoteVision] NoteVision IO: " + this.noteVisionIO.getClass().getSimpleName());

    }

    @Override
    public void periodic() {
        noteVisionIO.updateInputs(inputs);
        Logger.processInputs("NoteVision", inputs);
        var frameTargets = Arrays.asList(inputs.trackedNotes);
        var connections = new ArrayList<PhotonMemoryConnection>();
        noteMemories.forEach(
            (memory) -> frameTargets.forEach(
                (target) -> {
                    if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.get()) {
                        connections.add(new PhotonMemoryConnection(memory, target));
                    }
                }
            )
        );
        connections.sort((a, b) -> (int) Math.signum(a.getDistance() - b.getDistance()));
        var unusedMemories = new ArrayList<>(noteMemories);
        var unusedTargets = new ArrayList<>(frameTargets);
        while(!connections.isEmpty()) {
            var confirmedConnection = connections.get(0);
            confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.photonFrameTarget);
            confirmedConnection.memory.updateConfidence();
            unusedMemories.remove(confirmedConnection.memory);
            unusedTargets.remove(confirmedConnection.photonFrameTarget);
            connections.removeIf((connection) -> 
                connection.memory == confirmedConnection.memory
                ||
                connection.photonFrameTarget == confirmedConnection.photonFrameTarget
            );
        }
        unusedMemories.forEach((memory) -> {
            if(SUB_Drivetrain.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) > 1) {
                memory.decayConfidence(1);
            }
        });
        unusedTargets.forEach((target) -> noteMemories.add(target));
        noteMemories.removeIf((memory) -> memory.confidence <= 0);
        noteMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()));

        if(optIntakeTarget.isPresent() && optIntakeTarget.get().confidence < detargetConfidenceThreshold.get()) {
            optIntakeTarget = Optional.empty();
        }
        if(optIntakeTarget.isEmpty() || !intakeTargetLocked) {
            optIntakeTarget = noteMemories.stream().filter((target) -> target.getPriority() >= acquireConfidenceThreshold.get()).sorted((a,b) -> (int)Math.signum(b.getPriority() - a.getPriority())).findFirst();
        }
        
        // Logger.recordOutput("Vision/Note/Photon Frame Targets", frameTargets.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note/Note Memories", noteMemories.stream().map(TrackedNote::toASPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note/Note Confidence", noteMemories.stream().mapToDouble((note) -> note.confidence).toArray());
        Logger.recordOutput("Vision/Note/Note Priority", noteMemories.stream().mapToDouble(TrackedNote::getPriority).toArray());
        Logger.recordOutput("Vision/Note/Target", optIntakeTarget.map(TrackedNote::toASPose).map((a) -> new Pose3d[]{a}).orElse(new Pose3d[0]));
        Logger.recordOutput("Vision/Note/Locked Target", optIntakeTarget.filter((a) -> intakeTargetLocked).map(TrackedNote::toASPose).map((a) -> new Pose3d[]{a}).orElse(new Pose3d[0]));
        intakeTargetLocked = false;
    }

    public DoubleSupplier applyDotProduct(Supplier<ChassisSpeeds> joystickFieldRelative) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = SUB_Drivetrain.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var joystickSpeed = joystickFieldRelative.get();
            var joy = new Translation2d(joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
            var throttle = targetRelRobotNormalized.toVector().dot(joy.toVector());
            return throttle;
        }).orElse(0.0);
    }

    public LazyOptional<ChassisSpeeds> getAutoIntakeTransSpeed(DoubleSupplier throttleSupplier) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = SUB_Drivetrain.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var finalTrans = targetRelRobotNormalized.times(throttleSupplier.getAsDouble());
            return new ChassisSpeeds(finalTrans.getX(), finalTrans.getY(), 0);
        });
    }

    public LazyOptional<Translation2d> autoIntakeTargetLocation() {
        return () -> optIntakeTarget.map((target) -> {
            intakeTargetLocked = true;
            return target.fieldPos;
        });
    }

    public boolean hasTarget() {
        return optIntakeTarget.isPresent();
    }

    public void clearMemory() {
        noteMemories.clear();
        optIntakeTarget = Optional.empty();
    }

    public Command autoIntake(DoubleSupplier throttle, SUB_Drivetrain drive, SUB_Intake intake) {
        return 
            drive.fieldRelative(getAutoIntakeTransSpeed(throttle).orElseGet(ChassisSpeeds::new))
            .alongWith(
                drive.pointTo(autoIntakeTargetLocation(), () -> Drivetrain.intakeSide)
            )
            .onlyWhile(() -> !intake.intakeHasNote())
            .withName("Auto Intake")
        ;
    }

    private static record PhotonMemoryConnection(TrackedNote memory, TrackedNote photonFrameTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(photonFrameTarget.fieldPos);
        }
    }

    public static class TrackedNote implements StructSerializable {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedNote(Translation2d fieldPos, double confidence) {
            this.fieldPos = fieldPos;
            this.confidence = confidence * confUpdatingFilteringFactor.get();
        }

        public void updateConfidence() {
            confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        }

        public void updatePosWithFiltering(TrackedNote newNote) {
            this.fieldPos = fieldPos.interpolate(newNote.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newNote.confidence;
        }

        public void decayConfidence(double rate) {
            this.confidence -= confidenceDecayPerSecond.get() * rate * 0.02;
        }

        public double getPriority() {
            var pose = SUB_Drivetrain.getInstance().getPose();
            var FORR = fieldPos.minus(pose.getTranslation());
            var rotation = pose.getRotation().minus(Drivetrain.intakeSide);
            return 
                confidence * priorityPerConfidence.get() *
                VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit()) + 
                FORR.getNorm() * priorityPerDistance.get()
            ;
        }

        public Pose3d toASPose() {
            return new Pose3d(new Translation3d(fieldPos.getX(), fieldPos.getY(), Units.inchesToMeters(1)), new Rotation3d());
        }

        public static final TrackedNoteStruct struct = new TrackedNoteStruct();
        public static class TrackedNoteStruct implements Struct<TrackedNote> {
            @Override
            public Class<TrackedNote> getTypeClass() {
                return TrackedNote.class;
            }

            @Override
            public String getTypeString() {
                return "struct:TrackedNote";
            }

            @Override
            public int getSize() {
                return kSizeDouble + Translation2d.struct.getSize();
            }

            @Override
            public String getSchema() {
                return "Translation2d fieldPos;double confidence";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] {Translation2d.struct};
            }

            @Override
            public TrackedNote unpack(ByteBuffer bb) {
                var fieldPos = Translation2d.struct.unpack(bb);
                var confidence = bb.getDouble();
                return new TrackedNote(fieldPos, confidence);
            }

            @Override
            public void pack(ByteBuffer bb, TrackedNote value) {
                Translation2d.struct.pack(bb, value.fieldPos);
                bb.putDouble(value.confidence);
            }
        }
    }
}
