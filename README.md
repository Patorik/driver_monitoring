# Driver monitoring

This software contains a ROS2 system which is able to analyse video data of vehicle drivers.

## Requirements
Every requirements of the nodes are stored in the *src/<package_name>/requirements.txt* file.

## Bag files

## YOLOv7 networks
[**YOLOv7**](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt)
[**YOLOv7-tiny**](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-tiny.pt)

## Run the program
1. Use `colcon build` to build the packages.
2. Use `source install/setup.bash` to source the package.
3. Use `ros2 run <package_name> <node_name>` to run the nodes.
