Instead of creating a map, the 2D SLAM with LiDAR gives the distance measured in the form of a ROS LaserScan message such as /scan. 

Firstly, check if ROS driver for the LiDAR we are using exists, if so, use that. If not, we gotta write code for that as well. 

The MAP for 2D slam is just an occupancy matrix like those seat in flight bookings, 
-> vacant (0)
-> occupied (1)
-> unknown/unexplored 

We might run into pits/craters though which might be a problem as LiDAR is not very good at detecting the holes as it scans horizontally. So this would help with the boulder type of obstacles. 

# TRANSFORM

**Map** -  Where the rover is in the arena 

**ODOM** - How much and how the rover has moved. For example, it has moved 20m from starting point.   Where the rover is relative to its starting position.   ((Pos)start)rover

**Base_Link** -  Represents the rover itself.    Kind of like an unspoken coordinate frame (x,y,z)   x  -> forward 
y  -> left/right
z  -> upward

Where are things relative to the rover itself   ((Pos)everything)rover 

**LiDAR**  -> **sends data such as orientation and distance** -> **ROS packages it into** -> sensor_msgs/LaserScan -> **published as** /scan  -> **sent to slam** 


# 3D SLAM

***RGB*** - Shows the pixels and how exactly things look like. For example, it says “there is a rock.”

***Depth*** - Shows the distance between each pixel which is the depth value so basically, how far away everything is. For example, it says “it is 20m away”

RGB + Depth gives what object and how far it is. “The rock is 20m away”.

All of this is in the form of a 2D matrix and point cloud converts this into 3D. 

***Camera calibration*** - Focal length, principal point, anything that may distort the images such as the edges of the lenses etc and the dimensions. All of these may affect it so we calculate relative to this and make this our zero (It is like normalizing an error, “Zero” may not always be the standard or ideal). 

***IMU*** -   Accelerometer + Gyroscope (angular velocity, pitch, roll, yaw) + Magnetometer (magnetic field direction). IMU is needed incase there were speed bumps and the rover tilts making the LiDAR tilt. 

Since last time it was mentioned that camera jitter and vibration issue persisted, IMU would help as well (along with rigid, stiff bracket, aluminium with it being as low as possible). 

***CAC rulebook states that we cannot use the navigation aspect of IMU (Compass and GPS). We need to turn that off.*** 


# WHY  SLAM  TOOLBOX?

-> It is specifically made for 2D and ROS2.    

-> It has features related to odometry, laserscans, optimisation (pose graph) and also publishes the map (2D occupancy matrix) and pose (x,y,orientation). 

-> Can integrate it with NAV2

-> Can save and reload maps

-> Supports loop closure    

## ALTERNATIVES: 

***Cartographer (Google)*** - though it is not as straightforward when it comes to integration with NAV2. 


# WHY  RTAB  MAP ?

-> It supports stereo, RGB-D and LiDAR data. 

-> It can generate 3D maps and point clouds as well. 

-> It even represents occupancy.

-> We cannot use SLAM TOOLBOX as it doesn’t account for the information the real sense cameras can provide.

-> Can be run on both CPU and GPU

## ALTERNATIVES: 

**ORB SLAM 3**- Also supports stereo, RGB-D and visual intertial data but it does not create 3D maps. It creates a sparse map. Run on CPU. 


**NVIDIA ISAAC ROS VISUAL SLAM** - Also supports stereo, visual and IMU data but it maps the visual slam or odometry, not 3D. Can only be run on NVIDIA GPU. This would help as the ROS2 team is focussing on camera timing and the target of 30Hz and tight synchronization.

# PERFORMANCE : 

As for the performance, I cannot find any information on it so we need to measure it and checking things like the camera and depth resolution, fps, parameters for mapping and mainly, whether we are using IMU or not. 

The ISAAC  CuVSLAM one reportedly has a runtime of 7ms on Jetson AGX Xavier and ORB-SLAM2 has 60ms but since we would be using Jetson Orin, I don’t think we can consider that.  
