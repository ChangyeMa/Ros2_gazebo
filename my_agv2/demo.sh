# lift
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,-0.8,-0.1,0.3]}'

kill
# tilt
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,0.0,-0.1,0.3]}'

kill
# back to top
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,-2,-0.1,0.3]}'

kill
# unload_1
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,-0.8,0.1,0.3]}'
kill

# unload_2
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,-0.8,0.1,-0.3]}'
kill

# load_1
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,-0.8,-0.1,-0.3]}'
kill

# load_2
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [0.25,-0.6,-0.1,0.3]}'
kill

# back to low
# load_2
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "row", size: 4, stride: 4}], data_offset: 0}, data: [-0.2,0.0005,-0.1,0.3]}'
kill
