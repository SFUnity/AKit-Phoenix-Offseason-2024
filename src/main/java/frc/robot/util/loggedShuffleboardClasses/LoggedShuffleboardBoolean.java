package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedShuffleboardBoolean implements LoggedDashboardInput {
  private final String key;
  private boolean defaultValue;
  private boolean value;
  private SimpleWidget widget;

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
   * Creates a new LoggedShuffleboardBoolean, for handling a string input sent via NetworkTables.
   *
   * @param key The key for the boolean, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the button to appear in
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedShuffleboardBoolean(String key, String tab, boolean defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    this.widget = Shuffleboard.getTab(tab).add(key, defaultValue);
    periodic();
    Logger.registerDashboardInput(this);
  }

  public LoggedShuffleboardBoolean withWidget(WidgetType widgetType) {
    widget = widget.withWidget(widgetType);
    return this;
  }

  public LoggedShuffleboardBoolean withSize(int width, int height) {
    widget = widget.withSize(width, height);
    return this;
  }

  public LoggedShuffleboardBoolean withPosition(int x, int y) {
    widget = widget.withPosition(x, y);
    return this;
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(boolean defaultValue) {
    this.defaultValue = defaultValue;
  }

  /** Returns the current value. */
  public boolean get() {
    return value;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = widget.getEntry().getBoolean(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
