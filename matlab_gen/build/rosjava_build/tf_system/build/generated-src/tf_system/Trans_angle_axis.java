package tf_system;

public interface Trans_angle_axis extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "tf_system/Trans_angle_axis";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Vector3 translation\ngeometry_msgs/Vector3 angle_axis";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getTranslation();
  void setTranslation(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getAngleAxis();
  void setAngleAxis(geometry_msgs.Vector3 value);
}
