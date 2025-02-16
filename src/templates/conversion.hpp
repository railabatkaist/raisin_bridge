#include <@@PROJECT_NAME@@/msg/@@TYPE_SNAKE@@.hpp>
#include <include/@@PROJECT_NAME@@/msg/@@TYPE_SNAKE@@.hpp>

raisin::@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ to_raisin_msg(const @@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@::SharedPtr msg)
{
  raisin::@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ raisin_msg;
  return raisin_msg;
}

@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@::SharedPtr to_ros_msg(const raisin::@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@ msg)
{
  @@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@::SharedPtr ros_msg = std::make_shared<@@PROJECT_NAME@@::msg::@@TYPE_PASCAL@@>();
  return ros_msg;
}
