package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Constants;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedShuffleboardNumber
    implements LoggedShuffleboardComponentSimple<LoggedShuffleboardNumber> {
  private final String key;
  private double defaultValue;
  private double value;
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
   * Creates a new LoggedShuffleboardNumber, for handling a double input sent via NetworkTables.
   *
   * @param key The key for the double, published to "/Shuffleboard/{tab}/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the number to appear in
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedShuffleboardNumber(String key, String tab, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    value = defaultValue;
    widget = Shuffleboard.getTab(tab).add(key, defaultValue);
    entry = widget.getEntry();
    periodic();
    Logger.registerDashboardInput(this);
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
  }

  /** Sets the value. */
  public void set(double value) {
    entry.setDouble(value);
  }

  /** Returns the current value. */
  public double get() {
    return Constants.tuningMode ? value : defaultValue;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.getDouble(defaultValue);
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
