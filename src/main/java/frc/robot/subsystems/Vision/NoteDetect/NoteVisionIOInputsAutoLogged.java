package frc.robot.subsystems.Vision.NoteDetect;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class NoteVisionIOInputsAutoLogged extends NoteVisionIO.NoteVisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("TrackedNotes", trackedNotes);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    trackedNotes = table.get("TrackedNotes", trackedNotes);
  }

  public NoteVisionIOInputsAutoLogged clone() {
    NoteVisionIOInputsAutoLogged copy = new NoteVisionIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.trackedNotes = this.trackedNotes.clone();
    return copy;
  }
}
