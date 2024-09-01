package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Constants;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedShuffleboardBoolean
    implements LoggedShuffleboardComponentSimple<LoggedShuffleboardBoolean> {
  private final String key;
  private boolean defaultValue;
  private boolean value;
  private SimpleWidget widget;
  private GenericEntry entry;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(key, value);
        }

        public void fromLog(LogTable table) {
          value = table.get(key, defaultValue);
        }
      };

  /**
   * Creates a new LoggedShuffleboardBoolean, for handling a boolean input sent via NetworkTables.
   *
   * @param key The key for the boolean, published to "/Shuffleboard/{tab}/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the button to appear in
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedShuffleboardBoolean(String key, String tab, boolean defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    value = defaultValue;
    widget = Shuffleboard.getTab(tab).add(key, defaultValue);
    entry = widget.getEntry();
    periodic();
    Logger.registerDashboardInput(this);
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(boolean defaultValue) {
    this.defaultValue = defaultValue;
  }

  /** Sets the value. */
  public void set(boolean value) {
    entry.setBoolean(value);
  }

  /** Returns the current value. */
  public boolean get() {
    return Constants.tuningMode ? value : defaultValue;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.getBoolean(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }

  public SimpleWidget getWidget() {
    return widget;
  }

  public void setWidget(SimpleWidget widget) {
    this.widget = widget;
  }
}
