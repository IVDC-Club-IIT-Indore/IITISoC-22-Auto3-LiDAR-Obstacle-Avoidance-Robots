# LiDAR Obstacle Avoidance Robots

[Under construction... do not add to the final repo till someone removes this message]
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
1.Basic linux commands

2.ROS

3.Gazebo

4.

### Installation
'''sudo apt-get install ros-noetic-jackal-*
sudo apt-get install ros-noetic-velodyne-*'''
## Usage
Useful for any automated ground vehicles in warehouses
## How it works
It uses a 3D lidar (VLP 16) to gather data from its surroundings. The requirement of 3D data is to sense not just its paths but to also sense humans moving around.
## Roadmap
## Contributing
On the off chance that someone other than the judges or mentors see this, 

## Contact us

**Sairaj Loke:**
[LinkedIn](https://www.linkedin.com/in/sairaj-loke-24370b237)
[Github](https://github.com/SairajLoke)
WhatsApp: 93726 79958



**Akshit Raizada:**
[LinkedIn, ](https://www.linkedin.com/in/akshit-raizada-56a816228/)
[Github, ](https://github.com/AkshitRaizada)
WhatsApp: 96999 30412


**Abiroop Mohan:**
[LinkedIn, ](https://www.linkedin.com/in/abiroop-mohan-3145b322a/)
[Github, ](https://github.com/Abiroop)
WhatsApp: 73388 33401



**Aditya Guhagarkar:**
[LinkedIn, ](https://www.linkedin.com/in/aditya-guhagarkar/)
[Github, ](https://github.com/AG10GA)
WhatsApp: 97699 50455




## Acknowledgements
[1. https://github.com/koide3/hdl_graph_slam.git,](https://github.com/koide3/hdl_graph_slam.git)


[2. https://github.com/koide3/ndt_omp.git](https://github.com/koide3/ndt_omp.git)


[3. https://github.com/SMRT-AIST/fast_gicp.git](https://github.com/SMRT-AIST/fast_gicp.git)


[4. https://github.com/koide3/hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)



## Liscence
