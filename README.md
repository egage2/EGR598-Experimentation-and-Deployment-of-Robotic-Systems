# EGR598-Experimentation-and-Deployment-of-Robotic-Systems
Public reposity for EGR598 final project Spring 2023

Contains the two ROS2 node files the team generated for the final project

Final_find_blobs.py subscribes to the robot's /camera/preview/image, finds the red, blue, or green marker with using an HSV mask for each color, and then uses blob detection to find the markers, and publishes there color, position, and size data to a custom string message /blobs via color priority red>blue>green

Final_follow_blobs.py subscribes to the custom /blobs message, parsing only the first found blob, ie the blob of highest priority. From there it reads the data stored in the blob message, and does a combination of live plotting or raw vs filtered blob position, as well as linear and rotational motion commands that it publishes to the robot's /cmd_vel topic using a Twist type message. 

For red markers the operation will live plot the marker position comparred the filtered output position, and follow the marker to keep it at a set distance while rotating with it.
 
For blue markers, the operation will live plot the marker position comparred to the filtered output postion, and follow the marker using only rotational commands, this should leave the robot affixed in place, and simply rotate left or right to keep the marker in the center of it's view
 
Finally for green markers, the operation will solely live plot the marker position comparred to the filtered output of the marker's found position. This is mainly used for troubleshooting the blob detection paramterers, as well as the kalman filter parameters to see what would work best for this application.
