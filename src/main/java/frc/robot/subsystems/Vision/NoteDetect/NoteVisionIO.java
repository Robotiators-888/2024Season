package frc.robot.subsystems.Vision.NoteDetect;

import frc.robot.subsystems.Vision.NoteDetect.NoteVision.TrackedNote;

public interface NoteVisionIO {
    public static class NoteVisionIOInputs {
        public boolean connected;
        public TrackedNote[] trackedNotes = new TrackedNote[0];
    }

    public default void updateInputs(NoteVisionIOInputs inputs) {}
}
