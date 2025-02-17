raisin::@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ to_raisin_msg(const @@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@::SharedPtr ros_msg)
{
  raisin::@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ raisin_msg;
  @@CONVERSION_TO_RAISIN@@
  
  return raisin_msg;
}

@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ to_ros_msg(const raisin::@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@::SharedPtr raisin_msg)
{
  @@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ ros_msg;
  @@CONVERSION_TO_ROS@@

  return ros_msg;
}

