package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.networktables.GenericEntry;
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
   * Creates a new LoggedShuffleboardBoolean, for handling a string input sent via NetworkTables.
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

  /**
   * Sets the type of widget used to display the data. If not set, the default widget type will be
   * used.
   *
   * @param widgetType the type of the widget used to display the data
   */
  public LoggedShuffleboardBoolean withWidget(WidgetType widgetType) {
    widget = widget.withWidget(widgetType);
    return this;
  }

  /**
   * Sets the size of this component in the tab. This has no effect if this component is inside a
   * layout.
   *
   * @param width how many columns wide the component should be
   * @param height how many rows high the component should be
   */
  public LoggedShuffleboardBoolean withSize(int width, int height) {
    widget = widget.withSize(width, height);
    return this;
  }

  /**
   * Sets the position of this component in the tab. This has no effect if this component is inside
   * a layout.
   *
   * @param columnIndex the column in the tab to place this component
   * @param rowIndex the row in the tab to place this component
   */
  public LoggedShuffleboardBoolean withPosition(int columnIndex, int rowIndex) {
    widget = widget.withPosition(columnIndex, rowIndex);
    return this;
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
    return value;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.getBoolean(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
