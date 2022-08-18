# LiDAR Based Obstacle Avoidance Robots

Github Repo Under Development/Improvement  .
Suggestions are welcomed.


This repository was written for the Indian Institute of Technology Indore Summer of Code 2022. Kindly check the contributing section if you're not an IITI alumni.

Mentors:  [Kshitij Bhat](https://github.com/KshitijBhat),  [Bhavya Dalal](https://github.com/dalalbhavya)

Members :

-   [Sairaj Loke](https://github.com/SairajLoke)
-   [Akshit Raizada](https://github.com/AkshitRaizada)
-   [Abiroop Mohan](https://github.com/Abiroop)
-   [Aditya Guhagarkar](https://github.com/AG10GA)

## Table of Contents

- **A. Project  Description ( Problem Statement)**
 - **B. Software Stack And Environment/Robot used**
 - **C. Working Details**
    - 1.Overall Structure
	- 2.Lidar and SLAM 
	- 3.People Tracking
	- 4.Planning & Obstacle avoidance
 - **D. Getting Started** 
	 - Prerequisites and dependencies
	 - Running 3D SLAM
	 - Running HDL People Tracking
	 - Running Final Simulation
 - **E. References**
 - **F. What Next?** 

# A.  Project Description

Large Warehouses (such as those of Amazon) are highly dynamic environments with many moving objects. Autonomous Mobile Robots (AMRs) with 3D LiDARs and other sensors are deployed to move goods around the warehouse for efficient material movement in a robust and fast manner. Since there are also people working around, these advanced robots have to perceive the environment, make intelligent decisions and make judgments of static and dynamic obstacles (or other robots) around it to avoid collisions and hazards. LiDAR-based obstacle avoidance and dynamic tracking systems must be deployed with a robust decision-making architecture.

### Specifications:

-   Any 3D LiDAR may be used (_Ouster OS_  series or  _Velodyne Puck_).
-   Assume the robots are slow-moving.
-   A C++ based collision avoidance and planning module is preferred for such high-speed and high-accuracy tasks.
-   A few scenarios may be simulated with the developed software stack.

# B. Software Stack and Environment/Robot used

-   **ROS Noetic**  allows developers to easily simulate their robot in any environment, before deploying anything in the real world. This version is specific to  **Ubuntu 20.04**  and the above code has only been tested with these conditions. 
  
-   **Gazebo 11**  brings a a complete toolbox of development libraries and cloud services to make simulation easy.
  -  **Rviz, Rqt** visualization tools.


### LiDAR Model

This selection was primarily to receive accurate simulation data as well as ensuring that the computation times were low. The OS Ouster series involved LiDARs having up to 128 channels which were slowing down the computation and were running at about 1-2 fps on our computers without even enabling the navigation stack. Due to our use case being a warehouse, 16 and 32 channel LiDARs were ideal.

This ensures that our simulations can run on Gazebo. In the end, we selected VLP-16 due to the availability of some very cool datasets(like  [this](https://github.com/TixiaoShan/Stevens-VLP16-Dataset)) and compatibility with our model and certain nodes shown ahead.

### Robot

We split into 2 teams and tried to implement LiDARs on certain candidate models like Turtlebot3, Turtlebot3 Friends Series and Clearpath Jackal. Despite looking very interesting, the Turtlebot3 Friends Series either didn't have space for LiDARs or were not fit for work in a warehouse environment.

Out of the remaining models,  **Clearpath Jackal**  was clearly suited for higher speed tasks and had a better control system. It also had seamless integration with Velodyne LiDARs.
![image](https://user-images.githubusercontent.com/104747561/185407777-3ed8f264-b9c3-49b5-8133-6c5426496f7e.png)

### Environment used


We started off with a simple framework and developed our algorithms to test navigation with static objects.
To further challenge our robot we tested it in various complex environments such as a cafe, a hospital and finally the warehouse. In all of these environments, we used actors to simulate dynamic obstacles.
In gazebo there are two types of objects- actors and models. Models are perfect for simulating static objects as we can assign collision properties to it. Actors on the other hand can be given a trajectory and are ideal for dynamic objects.
Models can be moved using custom plugins but these are way harder to execute. We faced issues with the actors as they couldn't be identified by the lidar.
 This was resolved
by forcing GPU settings ON by modifying the description packages. As a result we essentially tricked it into interacting with the actors.


![](https://powerpoint.officeapps.live.com/pods/GetClipboardImage.ashx?Id=3bebb057-49e0-4d3f-81e7-f21e233b8871&DC=PSG4&pkey=bef5291f-78b9-425d-ac8a-ae338eb73aff&wdwaccluster=PSG4)


## C. Working Details

### C1. Overall Structure

![](https://powerpoint.officeapps.live.com/pods/GetClipboardImage.ashx?Id=a259094c-f6bf-45a5-bec1-7b77bd9d8b3d&DC=PSG4&pkey=f9c3f0c9-ee6e-495f-a2e7-2d53841be2c3&wdwaccluster=PSG4)



### C2. Lidar And SLAM 


[![Compute Graph](https://camo.githubusercontent.com/25024126d981bc56db09652a25db0eb1647c956f72927a19dc5467005d618bba/68747470733a2f2f6d656469612e646973636f72646170702e6e65742f6174746163686d656e74732f3939383931303839393639333833303134362f3939383931303939383631323239313734342f726f7367726170682e706e673f77696474683d31333836266865696768743d353135)](https://camo.githubusercontent.com/25024126d981bc56db09652a25db0eb1647c956f72927a19dc5467005d618bba/68747470733a2f2f6d656469612e646973636f72646170702e6e65742f6174746163686d656e74732f3939383931303839393639333833303134362f3939383931303939383631323239313734342f726f7367726170682e706e673f77696474683d31333836266865696768743d353135)  

We have used the Gmapping package available .
Attempt was made for Google Cartographer as well , but coluldn't enable it properly. Shall try it again soon.

The above is the compute graph excluding the 3D SLAM (since the mapping is only run once).

After selection, we simulated the LiDAR using 4-5 methods and the official implementation of Velodyne LiDAR(jackal_velodyne) gave us the most customizable form through a .xacro file. Thus, we can simulate different LiDARs through this library but let's stick to the one decided above since we have its datasets for training for object avoidance.

The Rviz configuration for the simulation was heavily modified to accurately plot the captured data including the robot model and its radius.

[![Model](https://camo.githubusercontent.com/a5f6deb9dfb19e1c6ea8fc04227eff90655a47f9706182e0fbb167d36a45f3af/68747470733a2f2f6d656469612e646973636f72646170702e6e65742f6174746163686d656e74732f3939383931303839393639333833303134362f3939383931343333333935363339353131392f7276697a5f73637265656e73686f745f323032325f30375f31392d31365f35375f35352e706e673f77696474683d363636266865696768743d363239)](https://camo.githubusercontent.com/a5f6deb9dfb19e1c6ea8fc04227eff90655a47f9706182e0fbb167d36a45f3af/68747470733a2f2f6d656469612e646973636f72646170702e6e65742f6174746163686d656e74732f3939383931303839393639333833303134362f3939383931343333333935363339353131392f7276697a5f73637265656e73686f745f323032325f30375f31392d31365f35375f35352e706e673f77696474683d363636266865696768743d363239)

While capturing data, it was clear that we need a high-speed navigation system. 3D maps require a large amount of computation to navigate while also being sort of redundant for AGVs which aren't moving in the z direction(like our work case). Despite the focus of the problem statement being low computation obstacle avoidance, we wished to build a general purpose warehouse robot that can be modified for specific use cases.

It further required customized PointCloud filtering for each individual warehouse case which is against the vision of the project to make a generalized robot(also very cumbersome).

The 2D LIDAR at the front is setup for continous navigation of its surrounding as well as 2D mapping. We will be relying on 2D navigation maps to reduce computation needed to navigate the environment. This allows us to use the 2D navigation of the Jackal system which avoids all objects. The radius of the vehicle has been modified to ensure that turns with slight elevations are handled smoothly.

The 3D LiDAR is a very powerful tool that will be useful for object detection, recognition and avoidance(implementation after mid eval). 3D SLAM is used to map the 3D environment so that these maps are loaded into the system only upon reaching the destination which saves valuable computation time. These maps can also be used to generate 2D maps in case the warehouse has steps or other low-lying objects below the level of the 2D LIDAR. It is also essential for integration with warehouses using the HEBI Robotic Arm with this stack.

Basically it is needed for any functions with the z axis involved(which is most of the work in a warehouse).

### C3. People Tracking

We have also implemented a people tracking algorithm. But haven't been able to test if it can detect people effectively because Gazebo doesn't give accurate collision models of moving human actors. 

[ upload a vid of the detection of person  walking around ]

But here's a demo of our 3D mapping:-  [![image](https://user-images.githubusercontent.com/105885452/179753612-eada1c57-8798-45d4-a697-1e37cf63d195.png)](https://user-images.githubusercontent.com/105885452/179753612-eada1c57-8798-45d4-a697-1e37cf63d195.png)  (Left) Gazebo Simulation (Right) Corresponding 3D map - The vehicle was displaced from the position shown in the simulation... the yellow lines at the bottom are the LiDAR lines falling on the ground generated at the "circled spots". A larger .bag file is required to generate larger maps, we've haven't moved much in this demo so the file is small.

The 3D data is recorded in a .bag file and then converted into a format that can be accepeted by the 3D slam algorithm.

Thus, a combination 2D and 3D data are used to track and navigate any area. Despite having a limited use case, we have also tested it in more challenging environments(provided in presentation).


### C4.Path Planning And Obstacle Avoidance

 A star is used to navigate this grid and find a suitable path along this(it needs to be in a form in which the machine can turn smoothly).
 
Dynamic obstacle avoidance was clearly a challenging task even for relatively more experienced programmers. So we decided to look at cutting edge research in this field to understand the requirements and slowly developed an approach that we felt was fitting. Some of our major inspirations were as follows:-  [https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9649733](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9649733)  [https://arxiv.org/pdf/1706.09068.pdf](https://arxiv.org/pdf/1706.09068.pdf)  Among many of the research papers we read, one stood out among the rest...  [this](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8787346)  
We did extensive comparison to choose our local Planner 
![](https://powerpoint.officeapps.live.com/pods/GetClipboardImage.ashx?Id=3fc1ba15-2262-418a-a3a4-95ae4d529e23&DC=PSG4&pkey=a97abf76-3451-43a3-ae13-c97aa1603cbb&wdwaccluster=PSG4)
TEB Local Planner is clearly the most accurate for our use case.


We have made changes to base_local_planner.yaml for setup and inclusion of dynamic obstacles.
Following are some of the parameters changed for improving performance in our case: 

1)Dynamic obstacles are given greater inflation value
2)We even made the local path follow the global path more strictly by changing the weights and via points for the local trajectory .

3[Robot Footprint Mode](http://wiki.ros.org/teb_local_planner/Tutorials/Obstacle%20Avoidance%20and%20Robot%20Footprint%20Model)
"polygon”

4.Goal Tolerance Parameters ( min error in final obstacle and orientation)

5.min_obstacle_dist

6.inflation_dist ( larger than obstacle dist)

7.include_dynamic_obstacles


  
The Bottleneck about TEB is it's very high computational burden (-> limited local costmap size/resolution resp. robot size).


*NOTE:
We were also inspired by F1/10th cars which also require extremely fast computation and operate in similar algorithms and configurations.*




 In simple terms, we added a dynamic obstacle layer which provides a custom inflation based on their movement.  


# D. Getting Started

Prerequisites

Type the following commands in the terminal to install the required dependencies:-

```
sudo apt-get install ros-noetic-jackal-simulator
sudo apt-get install ros-noetic-jackal-*
sudo apt-get install ros-noetic-velodyne-*
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o
sudo pip install ProgressBar2
sudo apt-get install ros-noetic-velodyne
sudo apt-get install ros-noetic-velodyne-simulator
sudo apt-get install ros-noetic-geographic-info
sudo apt-get install ros-noetic-robot-localization
sudo apt-get install ros-noetic-twist-mux
sudo apt-get install ros-noetic-pointcloud-to-laserscan
sudo apt-get install ros-noetic-teb-local-planner
rosdep install teb_local_planner
sudo apt-get install ros-noetic-stage-ros

```

Kindly install the robotic arm from here if required in your warehouse:  [http://wiki.ros.org/hebi_cpp_api_examples/ClearpathJackal](http://wiki.ros.org/hebi_cpp_api_examples/ClearpathJackal)

### Installation

PLEASE INSTALL ALL DEPENDENCIES ACCORDING TO THESE STEPS,

WE HAVE MADE HEAVY MODIFICATIONS IN THE DEPENDENCIES, WHICH WILL NOT BE REFLECTED UNLESS DOWNLOADED FROM THIS REPOSITORY.

This includes melodic & kinetic packages which have been modified to be run on noetic systems.

Type the following commands in the terminal:-

```
mkdir IITISOC_ws
cd IITISOC_ws
git clone https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots
mv IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots src
cd src
git clone https://github.com/koide3/hdl_graph_slam
git clone https://github.com/koide3/ndt_omp
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
cd ..
catkin_make

```

make sure you source your workspace :) 
```
source ~/IITISOC_ws/devel/setup.bash

```


## Usage
**TO RUN 2D SLAM AND NAVIGATION (BASE STACK)**

After going inside the workspace run

```
cd IITISOC_ws
source devel/setup.bash
export JACKAL_LASER_3D=1
roslaunch jackal_velodyne gmapping_demo.launch
sudo chmod 777 /opt/ros/noetic/share/jackal_navigation/maps
cd /opt/ros/noetic/share/jackal_navigation/maps
rosrun map_server map_saver -f mymap

```

Recording of the base stack:  [https://docs.google.com/document/d/1PtidvDHwXN7AUR4w-XdMQeE_i0ZZn1mg5lUonYRKuww/edit](https://docs.google.com/document/d/1PtidvDHwXN7AUR4w-XdMQeE_i0ZZn1mg5lUonYRKuww/edit)

**TO RUN 3D SLAM**

Step 1 : Type the following in a terminal:-

```
roscore

```

Step 2: Run BASE STACK(shown above) in a different terminal

Step 3 : Open a new terminal and type the following:-

```
rosparam set use_sim_time true
rosbag record /mid/points

```

Step 4 : Open a new terminal and type the following:-

```
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_501.launch

```

Step 5 : Open a new terminal and type the following:-

```
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz

```

Step 6 : Open a new terminal and type the following:-

```
cd bagfiles
rosrun rosbag topic_renamer.py /mid/points PleasE.bag /filtered_points FinalE.bag
rosrun hdl_graph_slam bag_player.py FinalE.bag

```

Step 7 : To save the 3D map (replace "/full_path_directory" with the path to the folder where you want to save it)

```
rosservice call /hdl_graph_slam/save_map "resolution: 0.05 destination: '/full_path_directory/map.pcd'"

```

This map can be launched during runtime alongside hdl people tracking and used for all 3D purposes. We also recommend using this to assign coordinates to certain sections which can be used to transport goods autonomously using 2D navigation followed by a robotic arm placing the product(ie complete automation). This might also require a camera sensor to recognize the object but that it far beyond the scope of the PS so it might be added if we feel like revisiting this.

**TO RUN AMCL(MORE ACCURATE... DOESN'T SUPPORT 3D UNLESS RUN ALONGSIDE BASE STACK)**

Step 1 : Follow the steps in BASE STACK

Step 2 : Run the following in a new terminal

```
roslaunch jackal_navigation amcl_demo.launch

```

**TO RUN HDL PEOPLE TRACKING**  This step can be merged with the above by modifying the launch files. Has been tested with the base stack.

```
rosparam set use_sim_time true
roslaunch hdl_people_tracking hdl_people_tracking.launch

```

```
roscd hdl_localization/rviz
rviz -d hdl_localization.rviz

```

```
rosbag play --clock hdl_400.bag

```

**HOW TO REMOVE THE 2D LIDAR**  This process is not recommended. Most research papers we read supported the idea that high speed navigation required lower computation but we wanted to see if we can shave off 100 dollars from the robot without costing the computation. We soon understood why research wasn't too keen on this option. We have provided this section for use cases where the investment into the robot is very low and the environment is relatively simple. (also this part took a lot of effort so we wanted to show it)

We approached this problem by writing a node that converted 3D pointcloud to 2D laserscan. This ensured that data was captured and converted into relevant obstacles. Immediately we realized that it considers everything as an obstacle so even parts like door frames show up as a wall on the map. Necessary filtering was done using this repository  [https://github.com/ChengeYang/SLAM-with-Velodyne-Lidar-and-Jackal-UGV](https://github.com/ChengeYang/SLAM-with-Velodyne-Lidar-and-Jackal-UGV)  (It relies on 14.04's Jackal framework so we had to rewrite parts of it). Finally, we remapped /mid/points to /velodyne_points and hit enter only to find that the filtering was specific to their map and doesn't work well for other maps.

At the very least, we wanted to map our warehouse accurately but the program started throwing errors that tf had 2 unconnected trees. After a lot of bug seaching, we realized that the process was so heavy that it brought our program to 0.1 Hz refresh(We were running it on 10 Hz). The solution is to use a GPU but we didn't have any working version at hand. Further this showed us that this process is too computation heavy to practically run even with C++ nodes.

So we stuck to the research papers for navigation and decided to use 3D LiDAR for more practical purposes along the z axis like the robotic arm functionalities, HDL people tracking and inventory management.


### Finalized

-   3D SLAM Implementation (We finished it before time... might refine)
-   Person Identification (Implemented but hasn't been tested)
-   3D Object TrackingWe were also inspired by F1/10th cars which also require extremely fast computation and operate in similar ways.
-   Improved Path Planning using C++ code

# What Next? 

(random thoughts... might implement if we have time)

Note: Members(or mentors) can add any wacky ideas here through pull requests. If everyone agrees to implement it, we'll switch it to finalized.

-   Use person identification to make a person follower robot so that it can carry those boxes around... or do something similar to  [MIT's Jackal](https://www.youtube.com/watch?v=CK1szio7PyA)  [Might do 1st half... our algorithm behaves like MIT's Jackal so 2nd half done]
    
-   Use roswtf to handle error situations like collisions [Done using TEB plugin]
    
-   Merge projects with Robotic arm IITISOC team to make something like  [this](https://youtu.be/H-uSBO5e0_M)  (Robotic arm combined with person identification sounds like an machine that would be useful for the unavoidable AI takeover :D ) [Partially done]
    
-   Multiple Robot simulations [Done... will be shown in presentation]
    
-   Controlling Jackal using keyboard [Done]

- Adding Crane/Lift like structure to jackal
 which will help in carrying goods  in a warehouse
    

 # Contributing

On the off chance that someone outside IIT Indore finds this repository, kindly refrain from sending pull requests or code till the contest is over. You are free to ask about the implementation of the project or any errors encountered during installation. Feel free to contact any of us.

Members must contribute as required and the person with the latest working version or test machine will upload the code and send a pull request to the mentors.

# Contact Us

**Sairaj Loke:**  [![](https://camo.githubusercontent.com/a80d00f23720d0bc9f55481cfcd77ab79e141606829cf16ec43f8cacc7741e46/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4c696e6b6564496e2d3030373742353f7374796c653d666f722d7468652d6261646765266c6f676f3d6c696e6b6564696e266c6f676f436f6c6f723d7768697465)](https://www.linkedin.com/in/sairaj-loke-24370b237)  [![](https://camo.githubusercontent.com/fbc3df79ffe1a99e482b154b29262ecbb10d6ee4ed22faa82683aa653d72c4e1/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4769744875622d3130303030303f7374796c653d666f722d7468652d6261646765266c6f676f3d676974687562266c6f676f436f6c6f723d7768697465)](https://github.com/SairajLoke)  [![](https://camo.githubusercontent.com/571384769c09e0c66b45e39b5be70f68f552db3e2b2311bc2064f0d4a9f5983b/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f476d61696c2d4431343833363f7374796c653d666f722d7468652d6261646765266c6f676f3d676d61696c266c6f676f436f6c6f723d7768697465)](mailto:cse210001035@iiti.ac.in)

**Akshit Raizada:**  [![](https://camo.githubusercontent.com/a80d00f23720d0bc9f55481cfcd77ab79e141606829cf16ec43f8cacc7741e46/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4c696e6b6564496e2d3030373742353f7374796c653d666f722d7468652d6261646765266c6f676f3d6c696e6b6564696e266c6f676f436f6c6f723d7768697465)](https://www.linkedin.com/in/akshit-raizada-56a816228/)  [![](https://camo.githubusercontent.com/fbc3df79ffe1a99e482b154b29262ecbb10d6ee4ed22faa82683aa653d72c4e1/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4769744875622d3130303030303f7374796c653d666f722d7468652d6261646765266c6f676f3d676974687562266c6f676f436f6c6f723d7768697465)](https://github.com/AkshitRaizada)  [![](https://camo.githubusercontent.com/571384769c09e0c66b45e39b5be70f68f552db3e2b2311bc2064f0d4a9f5983b/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f476d61696c2d4431343833363f7374796c653d666f722d7468652d6261646765266c6f676f3d676d61696c266c6f676f436f6c6f723d7768697465)](mailto:me210003009@iiti.ac.in)  [![](https://camo.githubusercontent.com/5d03c86f6a75f7cbe80d135d9162fbf6dc46a31253cf30a8e9bb8279b4d574d3/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f547769747465722d3144413146323f7374796c653d666f722d7468652d6261646765266c6f676f3d74776974746572266c6f676f436f6c6f723d7768697465)](https://twitter.com/thugunb)

**Abiroop Mohan:**  [![](https://camo.githubusercontent.com/a80d00f23720d0bc9f55481cfcd77ab79e141606829cf16ec43f8cacc7741e46/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4c696e6b6564496e2d3030373742353f7374796c653d666f722d7468652d6261646765266c6f676f3d6c696e6b6564696e266c6f676f436f6c6f723d7768697465)](https://www.linkedin.com/in/abiroop-mohan-3145b322a/)  [![](https://camo.githubusercontent.com/fbc3df79ffe1a99e482b154b29262ecbb10d6ee4ed22faa82683aa653d72c4e1/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4769744875622d3130303030303f7374796c653d666f722d7468652d6261646765266c6f676f3d676974687562266c6f676f436f6c6f723d7768697465)](https://github.com/Abiroop)  [![](https://camo.githubusercontent.com/571384769c09e0c66b45e39b5be70f68f552db3e2b2311bc2064f0d4a9f5983b/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f476d61696c2d4431343833363f7374796c653d666f722d7468652d6261646765266c6f676f3d676d61696c266c6f676f436f6c6f723d7768697465)](mailto:abiroopmohan@gmail.com)

**Aditya Guhagarkar:**  [![](https://camo.githubusercontent.com/a80d00f23720d0bc9f55481cfcd77ab79e141606829cf16ec43f8cacc7741e46/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4c696e6b6564496e2d3030373742353f7374796c653d666f722d7468652d6261646765266c6f676f3d6c696e6b6564696e266c6f676f436f6c6f723d7768697465)](https://www.linkedin.com/in/aditya-guhagarkar/)  [![](https://camo.githubusercontent.com/fbc3df79ffe1a99e482b154b29262ecbb10d6ee4ed22faa82683aa653d72c4e1/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4769744875622d3130303030303f7374796c653d666f722d7468652d6261646765266c6f676f3d676974687562266c6f676f436f6c6f723d7768697465)](https://github.com/AG10GA)  [![](https://camo.githubusercontent.com/571384769c09e0c66b45e39b5be70f68f552db3e2b2311bc2064f0d4a9f5983b/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f476d61696c2d4431343833363f7374796c653d666f722d7468652d6261646765266c6f676f3d676d61696c266c6f676f436f6c6f723d7768697465)](mailto:ee210002005@iiti.ac.in)

([back to top](https://github.com/AkshitRaizada/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/tree/Files-added#top))

## [](https://github.com/AkshitRaizada/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/tree/Files-added#acknowledgements)Acknowledgements

1.  [https://github.com/koide3/hdl_graph_slam.git](https://github.com/koide3/hdl_graph_slam.git),
2.  [https://github.com/koide3/ndt_omp.git](https://github.com/koide3/ndt_omp.git)
3.  [https://github.com/SMRT-AIST/fast_gicp.git](https://github.com/SMRT-AIST/fast_gicp.git)
4.  [https://github.com/koide3/hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)
