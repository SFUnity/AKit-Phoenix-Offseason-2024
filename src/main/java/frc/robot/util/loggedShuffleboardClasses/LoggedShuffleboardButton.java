package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedShuffleboardButton implements LoggedDashboardInput {
  private final String key;
  private boolean defaultValue;
  private boolean value;
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
   * @param key The key for the boolean, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  public LoggedShuffleboardButton(String key, String tab) {
    this(key, tab, false);
  }

  /**
   * Creates a new LoggedShuffleboardBoolean, for handling a string input sent via NetworkTables.
   *
   * @param key The key for the boolean, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the button to appear in
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedShuffleboardButton(String key, String tab, boolean defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    this.entry =
        Shuffleboard.getTab(tab)
            .add(key, defaultValue)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    periodic();
    Logger.registerDashboardInput(this);
  }
  // TODO figure out better way to do position and size

  /**
   * Creates a new LoggedShuffleboardBoolean, for handling a string input sent via NetworkTables.
   *
   * @param key The key for the boolean, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the button to appear in
   * @param defaultValue The default value if no value in NT is found.
   * @param width The width of the button
   * @param height The height of the button
   */
  public LoggedShuffleboardButton(
      String key, String tab, boolean defaultValue, int width, int height) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    this.entry =
        Shuffleboard.getTab(tab)
            .add(key, defaultValue)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(width, height)
            .getEntry();
    periodic();
    Logger.registerDashboardInput(this);
  }

  /**
   * Creates a new LoggedShuffleboardBoolean, for handling a string input sent via NetworkTables.
   *
   * @param key The key for the boolean, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the button to appear in
   * @param defaultValue The default value if no value in NT is found.
   * @param width The width of the button
   * @param height The height of the button
   */
  public LoggedShuffleboardButton(
      String key, String tab, boolean defaultValue, int width, int height, int column, int row) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    this.entry =
        Shuffleboard.getTab(tab)
            .add(key, defaultValue)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(width, height)
            .withPosition(column, row)
            .getEntry();
    periodic();
    Logger.registerDashboardInput(this);
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
      value = entry.getBoolean(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
