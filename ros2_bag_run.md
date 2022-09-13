# How to View rosbags

## Running the Bag

`ros2 bag <PATH_TO_FOLDER>`
example: `ros2 bag play bags/drive-square`

## rviz

Run `rvis2`

1. Add LaserScan topic
2. RobotModel
3. Odometry

If you can't see the robot, press "Reset"

## View odometry

Run `ros2 topic echo /odom` 
