# ROS 2 Course

## Overview
This repository contains Python templates for developing nodes as part of the ROS 2 course. Each template is designed to demonstrate and implement specific ROS 2 concepts, progressively building the students' understanding of ROS 2 nodes, topics, services, parameters, and control systems. 

The templates are numbered and organized to follow the course progression, starting with basic publisher-subscriber nodes and culminating in more complex control nodes with impedance control.

---

## Templates

1. **1_simple_publisher.py**  
   - A basic ROS 2 publisher node template.  
   - Demonstrates publishing messages to a topic.  

2. **2_simple_subscriber.py**  
   - A basic ROS 2 subscriber node template.  
   - Demonstrates subscribing to a topic and handling received messages.  

3. **3_stm_serial_communication_python.py**  
   - A Python-based serial communication template (not ROS2).  
   - Used to interface with STM hardware over a serial connection.  

4. **4_stm_serial_node_pub.py**  
   - A ROS 2 publisher node template for publishing STM hardware data.  
   - Demonstrates integration of serial communication with ROS topics.  

5. **5_stm_serial_node_pub_sub.py**  
   - A combined publisher and subscriber node template.  
   - Demonstrates publishing data from STM hardware and subscribing to commands.  

6. **6_simple_service.py**  
   - A basic ROS 2 service node template.  
   - Demonstrates the creation of a service server and client interaction.  

7. **7_simple_service_parameter.py**  
   - A service node template extended with ROS 2 parameters.  
   - Demonstrates how to use parameters to modify service behavior dynamically.  

8. **8_stm_control_node_poscontrol.py**  
   - A control node template for implementing position control.  
   - Demonstrates basic closed-loop control for STM hardware.  

9. **9_stm_control_node_impedance.py**  
   - A control node template for implementing impedance control.  
   - Demonstrates advanced control using virtual spring-damper models.

---

## Usage
Each template is designed to be a starting point for the exercises and deliverables in the course. Modify and extend the templates to implement the required functionality as per the instructions in the course materials.

1. Clone the repository to your Documents folder.
2. Navigate to the appropriate folder for your exercise.
3. Copy the template file into your ROS 2 package directory.
4. Modify the template to meet the deliverable requirements.

---

## Goals
These templates aim to:
- Provide a clear and structured learning path for ROS 2 development.
- Introduce key ROS 2 concepts like topics, services, and parameters.
- Demonstrate integration of hardware (STM) with ROS 2 nodes.
- Build a foundation for control systems and advanced robotics applications.

---

Happy learning!
