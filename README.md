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

## LiDAR Model

This selection was primarily to receive accurate simulation data alongside actually being able to run the simulation. The OS Ouster series involved LiDARs having up to 128 channels which weren't running on our Gazebo simulators. Due to our use case being a warehouse, 16 and 32 channel LiDARs can work very well.

This ensures that our simulations can run on Gazebo. In the end, we selected VLP-16 due to the availability of some very cool datasets(like [this](https://github.com/TixiaoShan/Stevens-VLP16-Dataset)) and compatibility with our model and certain nodes shown ahead. [Might be improved if we get access to a good GPU]

## Robot

We split into 2 teams and tried to implement LiDARs on certain candidate models like Turtlebot3, Turtlebot3 Friends Series and Clearpath Jackal. Despite looking very interesting, the Turtlebot3 Friends Series either didn't have space for LiDARs or were not fit for work in a warehouse environment.

Out of the remaining models, **Clearpath Jackal** was clearly suited for higher speed tasks and had a better control system. It also had seamless integration with Velodyne LiDARs.

<p align="right">(<a href="#top">back to top</a>)</p>

## Getting Started

### Prerequisites
### Installation
## Usage
## How it works
![Compute Graph](https://media.discordapp.net/attachments/998910899693830146/998910998612291744/rosgraph.png?width=1386&height=515)
The above is the compute graph excluding the 3D SLAM (since the mapping is only run once).

After selection, we simulated the LiDAR using 4-5 methods and the official implementation of Velodyne LiDAR(jackal_velodyne) gave us the most customizable form through a .xacro file. Thus, we can simulate different LiDARs through this library but let's stick to the one decided above since we have its datasets for training for object avoidance.

The Rviz configuration for the simulation was heavily modified to accurately plot the captured data and include the robot model.

![Model](https://media.discordapp.net/attachments/998910899693830146/998914333956395119/rviz_screenshot_2022_07_19-16_57_55.png?width=666&height=629)

While capturing data, it was clear that we need a high-speed navigation system. 3D maps require a large amount of computation to navigate while also being sort of redundant for AGVs which aren't moving in the z direction(like our work case).

The 2D LIDAR at the front is setup for continous navigation of its surrounding as well as 2D mapping. We will be relying on 2D navigation maps to reduce computation needed to navigate the environment. This allows us to use the 2D navigation of the Jackal system which avoids all objects, even if they aren't part of the original map adding it to an instance of the map which is later destroyed. (Navigation with moving objects in Gazebo will be tested after mid evaluation).

The 3D LiDAR is a very powerful tool that will be useful for object detection, recognition and avoidance(implementation after mid eval). 3D SLAM is used to map the 3D environment so that these maps are loaded into the system only upon reaching the destination which saves valuable computation time. These maps can also be used to generate 2D maps in case the warehouse has steps or other low-lying objects below the level of the 2D LIDAR. It can also be used to detect where a required good is placed and check for availability of space to keep a certain good.

As of now, we have implemented a 3D mapping algorithm and a people tracking algorithm. But haven't been able to test if it can detect people effectively due to an acute shortage of people in Gazebo. But here's a demo of our 3D mapping:-
![image](https://user-images.githubusercontent.com/105885452/179753612-eada1c57-8798-45d4-a697-1e37cf63d195.png)
(Left) Gazebo Simulation
(Right) Corresponding 3D map - The vehicle was displaced from the position shown in the simulation... the yellow lines at the bottom are the LiDAR lines falling on the ground generated at the "circled spots". A larger .bag file is required to generate larger maps, we've haven't moved much in this demo so the file is small.

The 3D data is recorded in a .bag file and then converted into a format that can be accepeted by the 3D slam algorithm.

Thus, a combination 2D and 3D data are used to track and navigate any area. Despite having a limited use case, we have also tested it in more challenging environments like the following:- (Many more .world files have been provided... modify line 7 in gmappingdemo.launch to try them)

(Put gifs of rviz simulations of different world, maybe even only 3d data)
## Roadmap
### Finalized
- [x] 3D SLAM Implementation (We finished it before time... might refine)
- [x] Person Identification (Implemented but hasn't been tested)
- [ ] 3D Object Tracking
- [ ] Improved Path Planning using C++ code
- [ ] Simplifying Installation Process (.bash files? , adding to repo?)

<div id="ideas"></div>

### Ideas

(random thoughts... might implement if we have time)

Note: Members(or mentors) can add any wacky ideas here through pull requests. If everyone agrees to implement it, we'll switch it to finalized.

- Provide set-up for IRL Jackal Installation

- Use person identification to make a person follower robot so that it can carry those boxes around... or do something similar to [MIT's Jackal](https://www.youtube.com/watch?v=CK1szio7PyA)

- Use roswtf to handle error situations like collisions

- Merge projects with Robotic arm IITISOC team to make something like [this](https://youtu.be/H-uSBO5e0_M) (Robotic arm combined with person identification sounds like an machine that would be useful for the unavoidable AI takeover :D )
 
## Contributing
On the off chance that someone other than the judges or mentors see this, 

[contributors-shield]: https://img.shields.io/github/contributors/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=informational&style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=blueviolet&style=for-the-badge
[forks-url]: https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/fork
[stars-shield]: https://img.shields.io/packagist/stars/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=%230000FF&logoColor=%230000FF&style=for-the-badge
[stars-url]: https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/stargazers
[issues-shield]: https://img.shields.io/github/issues-raw/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots?color=%23FF0000&style=for-the-badge
[issues-url]: https://github.com/IVDC-Club-IIT-Indore/IITISoC-22-Auto3-LiDAR-Obstacle-Avoidance-Robots/issues


[![](https://img.shields.io/badge/License-GPLv3-blue.svg)]()
