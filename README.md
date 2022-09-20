# warmup_project

Warm-Up Project for Comp Robo '22 by Daniel Park and Gabrielle Blake

## Abstract

The first project in this course, rightfully named the "Warmup Project", is meant to be an exercise to getting to know the ROS platform and working with the Neato robots. We developed a couple of behaviors for the robot: a basic TeleOp, driving in a square, wall following, person following, obstacle avoidance, and finally combining two of the behaviors: wall following and person tracking.

## Behaviors

For each behavior, we will describe the problem at a high-level, our strategy to approaching these solutions, and include any relevant diagrams that help explain our approach.

### Tele-Op

The tele-operation behavior is responsible for the robot being able to be controlled through keyboard commands. The approach used to achieve this behavior was to create a series of if statements such that depending on the key pressed, the robot would perform a different action.\newline 

In this case, the 'w' key would make the robot drive straight forward, the 'a' key would make it turn left in place, the 's' key would make it drive straight backwards, the 'd' key would make it turn right in place, the space bar was used to make the robot stop moving, and pressing 'control-c' would end the program. The robot drives straight and can turn through publishing messages to the `cmd_vel` ROS topic to control the robot's linear and angular velocity at any given time, depending on what key is pressed.

### Driving in a Square

As described by the project, this behavior drives in a meter-by-meter square as seen in the figure below. It does so by continuously going forward for a couple of seconds then turning ninety degrees to go in a square shape pattern.

For this behavior, we decided to use the timing to make the robot go in a square. While it would have been a better implementation on our end to control the robot's movement with odometry measurements since we had to hardcode the times we wanted to move forward and turn to accurately make a 1m by 1m square movement, we decided to use timing to save time for our next few sequences for wall and person following, which would take more time than this sequence. In addition, we wanted to use this motion to test out how to control motors in an automated setting (without a driver control). So using the previous Tele-Op commands, we just needed to control the motors going up and turning left after specific moments of time to finish this sequence.

### Wall Following

The wall-following behavior programs the robot drive parallel to walls when they are detected. We programmed the robot to drive along a wall at a distance of approximately 0.75 m from the wall.

In order to do this, we utilized the scan ROS topic to detect walls by looking for lines (which would be walls) in the robot's scans. We took the ranges of the scans at four angles, 5 and ten degrees above and below 90 degrees, so 80, 85, 95, and 100 degrees. To detect walls, we took the differences between the ranges at these angles and if they were sufficiently similar (all of the differences were under 0.5 m from each other), we assumed that there was a wall there. If a wall was detected, then the distance from the wall was evaluated using the most forward facing angle, which was 80 degrees. If the difference between the range being reported from 80 degrees and the ideal wall-distance of 0.75 m, was greater than 0.2 m the robot would turn to readjust depending on the sign of the difference. If the difference was positive, meaning the robot was driving too far or away from the wall, we would make it turn towards the wall by changing the angular velocity. If the difference was negative, meaning the robot was too close to the wall, we would make the angular velocity turn the robot away from the wall.

### Person Following

### Obstacle Avoidance

### Finite State Control

### Improvements and Finishing Remarks

