<img src="images/RAWB.png" />

# Introduction

The continued growth in collaborative efforts between humans and robots has a multitude of applications ranging from domestic household tasks such as lifting furniture to building habitats on other planets. As a result, the field of human-robot collaboration and interaction has become increasingly relevant in both industry and research. Our project, RAWB, wishes to delve deeper into the field of responsive collaboration, where feedback between both parties is exchanged in real time. Specifically, our primary objective is to be able to comfortably balance a static object in a tray suspended on one end by a human and on the other by a robotic arm in 3 degrees of freedom (DOF).

<br/>

# Design

### What design criteria must your project meet? What is the desired functionality?

RAWB's desired functionality is to make lifting the tray as easy and intuitive as possible while keeping the contents of the tray stable. When designing RAWB, our criteria were that it has to take relatively minute sensor inputs and output perfectly chosen actions to keep the tray and its contents steady. Due to the constantly changing human input, the robot will have to do this hundreds of times per second.

### Describe the design you chose.

Our design consists of a force sensor and camera for sensing the world, a non-rigid chain attachment for coupling to the tray, and a cascade of controllers PID from high to low level to actuate the robot.

### What design choices did you make when you formulated your design? What trade-offs did you have to make?

There are a few high-level elements that make up the system design. The sensors we used, the tray's attachment to the robot arm, and the underlying control software were all components of the design that needed to be chosen.

We settled on using both AR tag input from a USB camera and the force sensor on the end effector of the robot for sensor input. The force sensor allows us to detect forces from the hand in axes that are rigidly attached to the robot and thus wouldn't be detectable by viewing the system. The AR tags allow us to detect motion of the tray in axes that are not rigidly attached to the robot and thus can't easily be detected by the force sensor.

We built two attachments from the tray to the robot: a rigid clamp that constrains the tray in every axis but pitch, and a loose chain attachment that lets the tray move freely. The implementation of these attachments in described later, but each has benefits and drawbacks with respect to the rest of the design choices we made. Both attachments worked, but we settled on the chain attachment for most of our tests because the freedom it allows the human is nicer and more realistic in a real-world use case.

The robot's control systems were an interesting problem. The UR5e arm takes in joint angles and uses a proprietary low-level PID controller to move the arm to those angles as fast as it can. We designed a basic higher level controller to keep the arm's motion slow, smooth, and bounded within a certain region. On top of that, we have a high level PID controller that takes in sensor inputs and outputs a desired correction velocity.

### How do these design choices impact how well the project meets design criteria that would be encountered in a real engineering application, such as robustness, durability, and efficiency?

We believe we have chosen the best design for our real-world problem space. Our system is proven safe by the controllers we use, durable due to the strong but flexible attachment used and industrial-strength robot arm, and robust due to the sensor inputs we picked which work no matter what object the robot is carrying or its physical characteristics. Based off playing with the completed design, we can say that RAWB is pretty efficient and fun to use.

<br/>

# Implementation

### Describe any hardware you used or built. Illustrate with pictures and diagrams. What parts did you use to build your solution?

Our hardware was a mix of borrowed materials from Professor Francesco Borrelli’s Model Predictive Control (MPC) lab and custom designed and manufactured in the Jacobs and Invention Lab Makerspaces. The Universal Robotics (UR5e) robotic arm, and camera, was lent to us from the MPC lab. On the other hand, we designed and lasercut the tray and cart and printed the two end-effectors and end-effector mount for the arm. We designed two end-effectors with the intent of testing both a rigid C-gripper and a flexible, and as a result, underactuated, chain link design.

### Describe any software you wrote in detail. Illustrate with diagrams, flow charts, and/or other appropriate visuals. This includes launch files, URDFs, etc.

### How does your complete system work? Describe each step.

<br/>

# Results

Something here

<br/>

# Conclusion

Something here

<br/>

# Team

Something here

<img src="images/team.png" />

<br/>

# Additional Materials

Something here

{% include youtubePlayer.html id="jY_gpi_gsRI" %}
