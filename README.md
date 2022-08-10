MODIFY CONFIG OF GMAPPING

<div id="top"></div>

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![](https://img.shields.io/github/issues-pr-raw/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=important&style=for-the-badge)](https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/pulls)

# LiDAR Obstacle Avoidance Robots
This repository was written for the Indian Institute of Technology Indore Summer of Code 2022. Kindly check the contributing section if you're not an IITI alumni.

Mentors: [Kshitij Bhat](https://github.com/KshitijBhat), [Bhavya Dalal](https://github.com/dalalbhavya)

Members :  
- [Sairaj Loke](https://github.com/SairajLoke)
- [Akshit Raizada](https://github.com/AkshitRaizada)
- [Abiroop Mohan](https://github.com/Abiroop)
- [Aditya Guhagarkar](https://github.com/AG10GA)

[![](https://img.shields.io/badge/License-GPLv3-blue.svg)]()

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#project-description">Project Description</a>
      <ul>
        <li><a href="#specifications">Specifications</a></li>
      </ul>
    </li>
    <li>
      <a href="#selecting">Selecting</a>
      <ul>
        <li><a href="#software-stack">Software Stack</a></li>
        <li><a href="#lidar-model">LiDAR Model</a></li>
        <li><a href="#robot">Robot</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>    
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
      <a href="#usage">Usage</a>
    </li>
    <li>
      <a href="#how-it-works">How it works</a>
    </li>
    <li>
      <a href="#roadmap">Roadmap</a>
      <ul>
        <li><a href="#finalized">Finalized</a></li>
        <li><a href="#ideas">Ideas</a></li>
      </ul>
    </li>
    <li>
      <a href="#contributing">Contributing</a>
    </li>
    <li>
      <a href="#contact-us">Contact us</a>
    </li>
    <li>
      <a href="#acknowledgements">Acknowledgements</a>
    </li>
  </ol>
</details>

<!-- PROJECT DESCRIPTION -->
## Project Description 
Large Warehouses (such as those of Amazon) are
highly dynamic environments with many moving objects.
Autonomous Mobile Robots (AMRs) with 3D LiDARs and other
sensors are deployed to move goods around the warehouse for
efficient material movement in a robust and fast manner.
Since there are also people working around, these advanced
robots have to perceive the environment, make intelligent
decisions and make judgments of static and dynamic obstacles
(or other robots) around it to avoid collisions and hazards.
LiDAR-based obstacle avoidance and dynamic tracking systems
must be deployed with a robust decision-making architecture.

### Specifications:
- Any 3D LiDAR may be used (_Ouster OS_ series or _Velodyne
Puck_).
- Assume the robots are slow-moving.
- A C++ based collision avoidance and planning module is
preferred for such high-speed and high-accuracy tasks.
- A few scenarios may be simulated with the developed
software stack.
<p align="right">(<a href="#top">back to top</a>)</p>

## Selecting
### Software Stack

- **ROS Noetic** allows developers to easily simulate their robot in any environment, before deploying anything in the real world. This version is specific to **Ubuntu 20.04** and the above code has only been tested with these conditions.
This was coupled with additions like rQT and rViz to visualize relevant data.

- **Gazebo** brings a a complete toolbox of development libraries and cloud services to make simulation easy.

- Multiple command lines make execution of these statements much easier.

### LiDAR Model

This selection was primarily to receive accurate simulation data as well as ensuring that the computation times were low. The OS Ouster series involved LiDARs having up to 128 channels which were slowing down the computation and were running at about 1-2 fps on our computers without even enabling the navigation stack. Due to our use case being a warehouse, 16 and 32 channel LiDARs were ideal.

This ensures that our simulations can run on Gazebo. In the end, we selected VLP-16 due to the availability of some very cool datasets(like [this](https://github.com/TixiaoShan/Stevens-VLP16-Dataset)) and compatibility with our model and certain nodes shown ahead.

### Robot

We split into 2 teams and tried to implement LiDARs on certain candidate models like Turtlebot3, Turtlebot3 Friends Series and Clearpath Jackal. Despite looking very interesting, the Turtlebot3 Friends Series either didn't have space for LiDARs or were not fit for work in a warehouse environment.

Out of the remaining models, **Clearpath Jackal** was clearly suited for higher speed tasks and had a better control system. It also had seamless integration with Velodyne LiDARs.

<p align="right">(<a href="#top">back to top</a>)</p>

## Getting Started

### Prerequisites

Type the following commands in the terminal to install the required dependencies:-
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-navigation
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
http://wiki.ros.org/teb_local_planner/Tutorials/Configure%20and%20run%20Robot%20Navigation

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
If you ever encounter an error that the directory that you're trying to run is missing then run this:- (adding it to .bashrc might simplify this)
```
source ~/IITISOC_ws/devel/setup.bash
```

<p align="right">(<a href="#top">back to top</a>)</p>

## Usage
Insert commands to run the program and explain how to run its features here.

**TO RUN NAVIGATION**

After going inside the workspace run
```
cd IITISOC_ws
source devel/setup.bash
export JACKAL_LASER_3D=1
roslaunch jackal_velodyne gmapping_demo.launch
```


**TO RUN 3D SLAM**

Step 1 : Type the following in a terminal:-
```
roscore

```
Step 2: Run Navigation(shown above) in a different terminal

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

Step 6 :  Open a new terminal and type the following:-
```
cd bagfiles
rosrun rosbag topic_renamer.py /mid/points PleasE.bag /filtered_points FinalE.bag
rosrun hdl_graph_slam bag_player.py FinalE.bag
```

Step 7 : To save the 3D map (replace "/full_path_directory" with the path to the folder where you want to save it)
```
rosservice call /hdl_graph_slam/save_map "resolution: 0.05 destination: '/full_path_directory/map.pcd'"
```

<p align="right">(<a href="#top">back to top</a>)</p>

## How it works
![Compute Graph](https://media.discordapp.net/attachments/998910899693830146/998910998612291744/rosgraph.png?width=1386&height=515)
The above is the compute graph excluding the 3D SLAM (since the mapping is only run once).

After selection, we simulated the LiDAR using 4-5 methods and the official implementation of Velodyne LiDAR(jackal_velodyne) gave us the most customizable form through a .xacro file. Thus, we can simulate different LiDARs through this library but let's stick to the one decided above since we have its datasets for training for object avoidance.

The Rviz configuration for the simulation was heavily modified to accurately plot the captured data including the robot model and its radius.

![Model](https://media.discordapp.net/attachments/998910899693830146/998914333956395119/rviz_screenshot_2022_07_19-16_57_55.png?width=666&height=629)

While capturing data, it was clear that we need a high-speed navigation system. 3D maps require a large amount of computation to navigate while also being sort of redundant for AGVs which aren't moving in the z direction(like our work case). Despite the focus of the problem statement being low computation obstacle avoidance, we wished to build a general purpose warehouse robot that can be modified for specific use cases.

The 2D LIDAR at the front is setup for continous navigation of its surrounding as well as 2D mapping. We will be relying on 2D navigation maps to reduce computation needed to navigate the environment. This allows us to use the 2D navigation of the Jackal system which avoids all objects. The radius of the vehicle has been modified to ensure that turns with slight elevations are handled smoothly.

The 3D LiDAR is a very powerful tool that will be useful for object detection, recognition and avoidance(implementation after mid eval). 3D SLAM is used to map the 3D environment so that these maps are loaded into the system only upon reaching the destination which saves valuable computation time. These maps can also be used to generate 2D maps in case the warehouse has steps or other low-lying objects below the level of the 2D LIDAR. It is also essential for integration with warehouses using the HEBI Robotic Arm with this stack.

Basically it is needed for any functions with the z axis involved(which is most of the work in a warehouse).

We have also implemented a people tracking algorithm. But haven't been able to test if it can detect people effectively because Gazebo doesn't give accurate collision models of moving human actors. But here's a demo of our 3D mapping:-
![image](https://user-images.githubusercontent.com/105885452/179753612-eada1c57-8798-45d4-a697-1e37cf63d195.png)
(Left) Gazebo Simulation
(Right) Corresponding 3D map - The vehicle was displaced from the position shown in the simulation... the yellow lines at the bottom are the LiDAR lines falling on the ground generated at the "circled spots". A larger .bag file is required to generate larger maps, we've haven't moved much in this demo so the file is small.

The 3D data is recorded in a .bag file and then converted into a format that can be accepeted by the 3D slam algorithm.

Thus, a combination 2D and 3D data are used to track and navigate any area. Despite having a limited use case, we have also tested it in more challenging environments like the following:- (Many more .world files have been provided... modify line 7 in gmappingdemo.launch to try them)

(Put gifs of rviz simulations of different world, maybe even only 3d data)

========================================================================================
Dynamic obstacle avoidance was clearly a challenging task even for relatively more experienced programmers. So we decided to look at cutting edge research in this field to understand the requirements and slowly developed an approach that we felt was fitting. Some of our major inspirations were as follows:-
https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9649733
https://arxiv.org/pdf/1706.09068.pdf

It is clear that a robust framework was needed with a great deal of optimization of the current navigation stack. We also decided to add a dynamic layer that gives a greater inflation to dynamic obstacles based on their velocities.
We did a lot more research before settling on TEB local planner. The plugin allowed us to customize the navigation stack in real time which greatly simplified the process. It also provided a great framework for adding additional layers wherein we added a dynamic obstacles layer.

<p align="right">(<a href="#top">back to top</a>)</p>

## Roadmap

### Finalized
- [x] 3D SLAM Implementation (We finished it before time... might refine)
- [x] Person Identification (Implemented but hasn't been tested)
- [x] 3D Object Tracking
- [x] Improved Path Planning using C++ code
- [ ] Simplifying Installation Process (.bash files? , adding to repo?, use the src file properly and .bashrc)

<div id="ideas"></div>

### Ideas

(random thoughts... might implement if we have time)

Note: Members(or mentors) can add any wacky ideas here through pull requests. If everyone agrees to implement it, we'll switch it to finalized.

- Use person identification to make a person follower robot so that it can carry those boxes around... or do something similar to [MIT's Jackal](https://www.youtube.com/watch?v=CK1szio7PyA)

- Use roswtf to handle error situations like collisions

- Merge projects with Robotic arm IITISOC team to make something like [this](https://youtu.be/H-uSBO5e0_M) (Robotic arm combined with person identification sounds like an machine that would be useful for the unavoidable AI takeover :D )
- Mutiple Robot simulations
- Controlling Jackal using keyboard

<p align="right">(<a href="#top">back to top</a>)</p>



## Contributing

On the off chance that someone outside IIT Indore finds this repository, kindly refrain from sending pull requests or code till the contest is over. You are free to ask about the implementation of the project or any errors encountered during installation. Feel free to contact any of us.

Members must contribute as required and the person with the latest working version or test machine will upload the code and send a pull request to the mentors. 

(Note: I didn't know what to write in this section so I just wrote what I thought was relevant. If the mentors have any objections kindly let us know. If not, we'll remove this bracket during the next README update)

<p align="right">(<a href="#top">back to top</a>)</p>

## Contact Us

**Sairaj Loke:**
[![](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/sairaj-loke-24370b237)
[![](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/SairajLoke)
[![](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:cse210001035@iiti.ac.in)


**Akshit Raizada:**
[![](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/akshit-raizada-56a816228/)
[![](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/AkshitRaizada)
[![](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:me210003009@iiti.ac.in)
[![](https://img.shields.io/badge/Twitter-1DA1F2?style=for-the-badge&logo=twitter&logoColor=white)](https://twitter.com/thugunb)

**Abiroop Mohan:**
[![](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/abiroop-mohan-3145b322a/)
[![](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/Abiroop)
[![](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:abiroopmohan@gmail.com)

**Aditya Guhagarkar:**
[![](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/aditya-guhagarkar/)
[![](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/AG10GA)
[![](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:ee210002005@iiti.ac.in)

<p align="right">(<a href="#top">back to top</a>)</p>

## Acknowledgements

1. https://github.com/koide3/hdl_graph_slam.git,
2. https://github.com/koide3/ndt_omp.git
3. https://github.com/SMRT-AIST/fast_gicp.git
4. https://github.com/koide3/hdl_graph_slam

[contributors-shield]: https://img.shields.io/github/contributors/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=informational&style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=blueviolet&style=for-the-badge
[forks-url]: https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/fork
[stars-shield]: https://img.shields.io/github/stars/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=yellow&style=for-the-badge
[stars-url]: https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/stargazers
[issues-shield]: https://img.shields.io/github/issues-raw/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=%23FF0000&style=for-the-badge
[issues-url]: https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/issues

