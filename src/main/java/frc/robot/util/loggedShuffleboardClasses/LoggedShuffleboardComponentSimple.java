package frc.robot.util.loggedShuffleboardClasses;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public interface LoggedShuffleboardComponentSimple<T extends LoggedShuffleboardComponentSimple<T>>
    extends LoggedDashboardInput {

  /**
   * Gets the SimpleWidget associated with this component.
   *
   * @return the SimpleWidget
   */
  SimpleWidget getWidget();

  /**
   * Sets the SimpleWidget for this component.
   *
   * @param widget the SimpleWidget to set
   */
  void setWidget(SimpleWidget widget);

  /**
   * Sets the type of widget used to display the data. If not set, the default widget type will be
   * used.
   *
   * @param widgetType the type of the widget used to display the data
   */
  @SuppressWarnings("unchecked")
  public default T withWidget(WidgetType widgetType) {
    setWidget(getWidget().withWidget(widgetType));
    return (T) this;
  }

  /**
   * Sets custom properties for this component. Property names are case- and whitespace-insensitive
   * (capitalization and spaces do not matter).
   *
   * @param properties the properties for this component
   * @return this component
   */
  @SuppressWarnings("unchecked")
  public default T withProperties(Map<String, Object> properties) {
    setWidget(getWidget().withProperties(properties));
    return (T) this;
  }

  /**
   * Sets the size of this component in the tab. This has no effect if this component is inside a
   * layout.
   *
   * @param width how many columns wide the component should be
   * @param height how many rows high the component should be
   */
  @SuppressWarnings("unchecked")
  public default T withSize(int width, int height) {
    setWidget(getWidget().withSize(width, height));
    return (T) this;
  }

  /**
   * Sets the position of this component in the tab. This has no effect if this component is inside
   * a layout.
   *
   * @param columnIndex the column in the tab to place this component
   * @param rowIndex the row in the tab to place this component
   */
  @SuppressWarnings("unchecked")
  public default T withPosition(int columnIndex, int rowIndex) {
    setWidget(getWidget().withPosition(columnIndex, rowIndex));
    return (T) this;
  }
}
