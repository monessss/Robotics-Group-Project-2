# Robotics Group Project 2 - Group 32

## Project Introduction

This repository records the work of **Group 32** in the **Robotics Group Project 2** course of the Computer Science programme jointly developed by **Ocean University of China** and **Heriot-Watt University**.

Our team members are:

* **Ruihang Liu**
* **Xintong Jiang**

The purpose of this repository is to share our learning process, record our experimental results, and provide useful references for students who are also working on Arduino-based robotics projects.

In this project, we designed and tested an Arduino-based smart car. The main tasks of the course are divided into three parts:

1. **Light-Seeking Driving**
2. **Obstacle Avoidance Test**
3. **Line-Following Driving**

Among these three tasks, line following is the most challenging part because it requires stable sensor detection, motor speed control, and continuous adjustment during movement.

---

## Project Tasks

### 1. Light-Seeking Driving

In the first task, the car is required to move towards a light source.
The basic idea is to use light sensors to compare the light intensity on different sides of the car. According to the sensor readings, the car adjusts the speed of the left and right motors to move towards the brighter direction.

This task helps us understand:

* Basic sensor reading
* Motor control
* Direction adjustment
* Simple feedback control logic

---

### 2. Obstacle Avoidance Test

In the second task, the car is equipped with infrared sensors, which act like the "eyes" of the car.
The car needs to complete a one-minute obstacle avoidance test. During the test, it should detect obstacles in front of or around it and change direction in time to avoid collision.

This task focuses on:

* Infrared sensor detection
* Obstacle judgment
* Emergency stopping
* Turning and backing logic
* Stability during continuous movement

---

### 3. Line-Following Driving

The final and most difficult task is line following.
The car needs to follow a black line on the ground by using infrared tracking sensors. During movement, the car continuously reads the sensor values and adjusts the motor speeds to stay on the line.

This task is challenging because the car must deal with:

* Straight lines
* Curves
* Sharp turns
* Possible sensor noise
* Motor speed differences
* Loss of line detection

To improve performance, we tested different thresholds, motor speeds, and turning strategies.

---

## Hardware Used

The main hardware components used in this project include:

* Arduino Uno
* Motor driver module
* DC motors
* Infrared tracking sensors
* Infrared obstacle avoidance sensors
* Light sensors
* Battery module
* Robot car chassis
* Jumper wires and basic electronic components

---

## Repository Structure

```text
Group32-Robotics-Project-2/
│
├── README.md
│
├── src/
│   └── light_seeking.ino
│   └── obstacle_avoidance.ino
│   └── line_following.ino
│
├── images/
│   └── project_photos/
│
└── videos/
    └── test_results/
```

The `.ino` files are Arduino program files and can be opened directly with the Arduino IDE.

---

## How to Run

1. Open the corresponding `.ino` file in Arduino IDE.
2. Connect the Arduino Uno board to the computer.
3. Select the correct board type:

   ```text
   Arduino Uno
   ```
4. Select the correct serial port.
5. Upload the program to the Arduino board.
6. Place the car in the correct testing environment.
7. Start the test and observe the car's performance.

---

## Project Progress

| Task                  | Description                                         | Status                   |
| --------------------- | --------------------------------------------------- | ------------------------ |
| Light-Seeking Driving | The car moves towards the light source              | Completed / Testing      |
| Obstacle Avoidance    | The car avoids obstacles using infrared sensors     | Completed / Testing      |
| Line Following        | The car follows a black line using tracking sensors | Completed / Testing      |

---

## Problems and Improvements

During the development process, we met several problems, including:

* The two motors did not always run at the same speed.
* The car sometimes turned too sharply.
* The car could lose the line when facing sharp turns.
* Sensor readings were affected by the environment.
* The obstacle avoidance response needed to be fast enough during the one-minute test.

To solve these problems, we tried to:

* Adjust the left and right motor speeds separately.
* Test different sensor thresholds.
* Reduce unnecessary delay in the program.
* Improve the turning logic.
* Record test results and modify the code step by step.

---

## Learning Outcomes

Through this project, we improved our understanding of:

* Arduino programming
* Sensor-based robot control
* Basic embedded system design
* Motor driving logic
* Debugging through real-world testing
* Team cooperation and project documentation

This project also helped us realize that a robot program cannot only work in theory. It must be tested repeatedly in the real environment, because motor performance, sensor noise, ground material, and battery voltage can all influence the final result.

---

## Team Members

| Name          | Group    | Role                                   |
| ------------- | -------- | -------------------------------------- |
| Ruihang Liu   | Group 32 | Programming, testing, documentation    |
| Xintong Jiang | Group 32 |Programming，hardware setup, testing, documentation |

---

## Purpose of This Repository

This GitHub repository is created for:

* Recording our project progress
* Sharing our code and testing experience
* Summarizing problems and solutions
* Helping future students understand similar robotics tasks
* Preserving the final results of Group 32

This repository is mainly for learning and communication purposes.
