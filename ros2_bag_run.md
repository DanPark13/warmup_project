# How to View rosbags

## Install the Bag

`ros2 bag record /accel /bump /odom /cmd_vel /scan /stable_scan /projected_stable_scan /tf /tf_static -o bag-file-name`

Replace `bag-file-name` with a name

## Running the Bag

`ros2 bag <PATH_TO_FOLDER>`
example: `ros2 bag play bags/drive-square`

## rviz

Run `rviz2`

1. Add LaserScan topic
2. RobotModel
3. Odometry

If you can't see the robot, press "Reset"

## View odometry

Run `ros2 topic echo /odom` 

