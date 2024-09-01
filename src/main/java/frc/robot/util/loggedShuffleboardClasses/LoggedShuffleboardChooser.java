package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedShuffleboardChooser<V>
    implements LoggedShuffleboardComponentComplex<LoggedShuffleboardChooser<V>> {
  private final String key;
  private String selectedValue = null;
  private SendableChooser<String> sendableChooser = new SendableChooser<>();
  private Map<String, V> options = new HashMap<>();
  private Map<Integer, V> lastHasChangedValues = new HashMap<>();
  private ComplexWidget widget;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(key, selectedValue);
        }

        public void fromLog(LogTable table) {
          selectedValue = table.get(key, selectedValue);
        }
      };

  /**
   * Creates a new LoggedDashboardChooser, for handling a chooser input sent via NetworkTables.
   *
   * @param key The key for the boolean, published to "/Shuffleboard/{tab}/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param tab name of the Shuffleboard tab you want the button to appear in
   */
  public LoggedShuffleboardChooser(String key, String tab) {
    this.key = key;
    widget = Shuffleboard.getTab(tab).add(key, sendableChooser);
    periodic();
    Logger.registerDashboardInput(this);
  }

  /**
   * Creates a new LoggedDashboardChooser, for handling a chooser input sent via NetworkTables. This
   * constructor copies the options from a SendableChooser. Note that updates to the original
   * SendableChooser will not affect this object.
   *
   * @param key The key for the chooser, published to "/Shuffleboard/{tab}/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  @SuppressWarnings("unchecked")
  public LoggedShuffleboardChooser(String key, String tab, SendableChooser<V> chooser) {
    this(key, tab);

    // Get options map
    Map<String, V> options = new HashMap<>();
    try {
      Field mapField = SendableChooser.class.getDeclaredField("m_map");
      mapField.setAccessible(true);
      options = (Map<String, V>) mapField.get(chooser);
    } catch (NoSuchFieldException
        | SecurityException
        | IllegalArgumentException
        | IllegalAccessException e) {
      e.printStackTrace();
    }

    // Get default option
    String defaultString = "";
    try {
      Field defaultField = SendableChooser.class.getDeclaredField("m_defaultChoice");
      defaultField.setAccessible(true);
      defaultString = (String) defaultField.get(chooser);
    } catch (NoSuchFieldException
        | SecurityException
        | IllegalArgumentException
        | IllegalAccessException e) {
      e.printStackTrace();
    }

    // Add options
    for (String optionKey : options.keySet()) {
      if (optionKey.equals(defaultString)) {
        addDefaultOption(optionKey, options.get(optionKey));
      } else {
        addOption(optionKey, options.get(optionKey));
      }
    }
  }

  /** Adds a new option to the chooser. */
  public void addOption(String key, V value) {
    sendableChooser.addOption(key, key);
    options.put(key, value);
  }

  /** Adds a new option to the chooser and sets it to the default. */
  public void addDefaultOption(String key, V value) {
    sendableChooser.setDefaultOption(key, key);
    options.put(key, value);
  }

  /**
   * Returns the selected option. If there is none selected, it will return the default. If there is
   * none selected and no default, then it will return {@code null}.
   */
  public V get() {
    return options.get(selectedValue);
  }

  /**
   * Returns the internal sendable chooser object, for use when setting up dashboard layouts. Do not
   * read data from the sendable chooser directly.
   */
  public SendableChooser<String> getSendableChooser() {
    return sendableChooser;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      selectedValue = sendableChooser.getSelected();
    }
    Logger.processInputs(prefix, inputs);
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    V currentValue = get();
    V lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  public ComplexWidget getWidget() {
    return widget;
  }

  public void setWidget(ComplexWidget widget) {
    this.widget = widget;
  }
}
