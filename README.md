# cpp_snippets

ros2 run topic_tools transform /control/command/control_cmd --field data std_msgs/Float64 'std_msgs.msg.Float64(data=m.data)' --import std_msgs

ros2 run topic_tools transform /control/command/control_cmd --field data /a std_msgs/Float64 'std_msgs.msg.Float64(data=float(m))' --import std_msgs
