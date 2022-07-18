# LiDAR Obstacle Avoidance Robots

[Under construction... do not add to the final repo till someone removes this message]

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
<p align="right">(<a href="#top">back to top</a>)</p>

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

This ensures that our simulations can run on Gazebo. In the end, we selected VLP-16 due to the availability of some very cool datasets(like [this](https://github.com/TixiaoShan/Stevens-VLP16-Dataset)) and compatibility with our model and certain nodes shown ahead.

## Robot

We split into 2 teams and tried to implement LiDARs on certain candidate models like Turtlebot3, Turtlebot3 Friends Series and Clearpath Jackal. Despite looking very interesting, the Turtlebot3 Friends Series either didn't have space for LiDARs or were not fit for work in a warehouse environment.

Out of the remaining models, **Clearpath Jackal** was clearly suited for higher speed tasks and had a better control system. It also had seamless integration with Velodyne LiDARs.
