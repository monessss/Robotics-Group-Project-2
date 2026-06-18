# Robotics Group Project 2 - Group 32

## Project Introduction

This repository records the work of **Group 32** in the **Robotics Group Project 2** course of the Computer Science programme jointly developed by **Ocean University of China** and **Heriot-Watt University**.

Our team members are:

* **Ruihang Liu**
* **Xintong Jiang**

The purpose of this repository is to share our learning process, record our experimental results, and provide useful references for students who are also working on Arduino-based robotics projects.

In this project, we designed and tested an Arduino-based smart car. The main tasks of the course are divided into four parts:

1. **Light-Seeking Driving**
2. **Obstacle Avoidance Test**
3. **Line-Following Driving**
4. **I2C Sensor Demonstration and Lift Detection**

Among these tasks, line following is one of the most challenging parts because it requires stable sensor detection, motor speed control, and continuous adjustment during movement. The newly added I2C sensor demonstration task is also important because it combines multiple sensors, motor control, and real-time debugging in one program.

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

The third task is line following.

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

### 4. I2C Sensor Demonstration and Lift Detection

The newly added task is an I2C-based sensor demonstration program. This program connects several I2C devices and combines sensor reading with motor control.

The program uses:

* **ADS1115** to read three infrared tracking sensors
* **BME280** to read temperature, humidity, and air pressure
* **MPU-9250** to read acceleration and gyroscope data
* **L298N motor driver** to control the left and right motors

The main demonstration effect is:

1. When the robot is placed still on the ground, the wheels remain stopped.
2. When the robot is quickly lifted upward, the wheels start running forward.
3. When the robot is held in the air, the wheels keep running.
4. When the robot starts moving downward, the wheels stop.
5. The landing rebound is ignored to avoid false triggering.

This program does not use LED modules. The original LED pins are removed from the code, while the other interfaces remain unchanged.

This task helps us understand:

* I2C communication
* Multiple sensor integration
* Raw register reading from the MPU sensor
* Real-time motion detection
* Motor control based on acceleration changes
* Serial monitor debugging

---

## Hardware Used

The main hardware components used in this project include:

* Arduino Uno
* L298N motor driver module
* DC motors
* Infrared tracking sensors
* Infrared obstacle avoidance sensors
* Light sensors
* ADS1115 analog-to-digital converter module
* BME280 temperature, humidity, and pressure sensor
* MPU-9250 motion sensor
* Battery module
* Robot car chassis
* Jumper wires and basic electronic components

---

## Hardware Interface

### L298N Motor Driver

| Arduino Pin | L298N Pin | Function              |
| ----------- | --------- | --------------------- |
| D5          | ENA       | Right motor PWM       |
| D4          | IN1       | Right motor direction |
| D7          | IN2       | Right motor direction |
| D6          | ENB       | Left motor PWM        |
| D9          | IN3       | Left motor direction  |
| D10         | IN4       | Left motor direction  |

### I2C Bus

For Arduino Uno:

| Arduino Pin | I2C Function |
| ----------- | ------------ |
| A4          | SDA          |
| A5          | SCL          |

The I2C devices used in the sensor demonstration program are:

| Device   | I2C Address  | Function                                 |
| -------- | ------------ | ---------------------------------------- |
| ADS1115  | 0x48         | Read infrared sensor values              |
| BME280   | 0x76 or 0x77 | Read temperature, humidity, and pressure |
| MPU-9250 | 0x68         | Read acceleration and gyroscope data     |

### ADS1115 Sensor Channels

| ADS1115 Channel | Sensor                          |
| --------------- | ------------------------------- |
| A0              | Left infrared tracking sensor   |
| A1              | Center infrared tracking sensor |
| A2              | Right infrared tracking sensor  |

---

## Repository Structure

```text
Group32-Robotics-Project-2/
│
├── README.md
│
├── src/
│   ├── light_seeking.ino
│   ├── obstacle_avoidance.ino
│   ├── line_following.ino
│   └── i2c_sensor_lift_detection.ino
│

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

For the I2C sensor demonstration program, open the Serial Monitor and set the baud rate to:

```text
115200
```

At startup, the program scans the I2C bus. Normally, the following addresses should be detected:

```text
0x48  ADS1115
0x68  MPU-9250
0x76 or 0x77  BME280
```

If one of these addresses is missing, the wiring or sensor connection should be checked.

---

## Debugging Commands for I2C Sensor Demonstration

The I2C sensor demonstration program supports Serial Monitor commands for debugging.

### Show Help

```text
help
```

### Test Motors Manually

```text
motor 120 120
```

This command sets the left and right motor speeds to 120.

### Stop Motors

```text
stop
```

### Return to Automatic Lift Detection Mode

```text
auto
```

### Adjust Forward Speed

```text
speed 130
```

### Reverse Motor Direction

If the left motor runs in the wrong direction:

```text
reverseL 1
```

If the right motor runs in the wrong direction:

```text
reverseR 1
```

### Adjust Lift Detection Sensitivity

If the robot does not respond when lifted upward, reduce the lift threshold:

```text
lift 0.15
```

If the robot is triggered too easily, increase the lift threshold:

```text
lift 0.25
```

### Adjust Downward Detection Sensitivity

If the wheels do not stop when the robot is moved downward:

```text
down -0.10
```

If the wheels stop too easily while the robot is held in the air:

```text
down -0.20
```

### Recalibrate MPU Baseline

Before using lift detection, the robot should be kept still during startup. If the motion detection is unstable, recalibrate the MPU baseline:

```text
base
```

### Scan I2C Bus

```text
scan
```

This command is used to check whether the I2C devices are connected correctly.

---

## Project Progress

| Task                     | Description                                                                    | Status              |
| ------------------------ | ------------------------------------------------------------------------------ | ------------------- |
| Light-Seeking Driving    | The car moves towards the light source                                         | Completed / Testing |
| Obstacle Avoidance       | The car avoids obstacles using infrared sensors                                | Completed / Testing |
| Line Following           | The car follows a black line using tracking sensors                            | Completed / Testing |
| I2C Sensor Demonstration | The car reads multiple I2C sensors and controls motors based on lift detection | Completed / Testing |

---

## Problems and Improvements

During the development process, we met several problems, including:

* The two motors did not always run at the same speed.
* The car sometimes turned too sharply.
* The car could lose the line when facing sharp turns.
* Sensor readings were affected by the environment.
* The obstacle avoidance response needed to be fast enough during the one-minute test.
* The I2C devices needed correct wiring and address detection.
* The lift detection program required careful adjustment of acceleration thresholds.
* Motor direction could be different depending on the actual wiring.

To solve these problems, we tried to:

* Adjust the left and right motor speeds separately.
* Test different sensor thresholds.
* Reduce unnecessary delay in the program.
* Improve the turning logic.
* Use I2C scanning to check sensor connections.
* Add Serial Monitor commands for real-time debugging.
* Recalibrate the MPU baseline before testing motion detection.
* Record test results and modify the code step by step.

---

## Learning Outcomes

Through this project, we improved our understanding of:

* Arduino programming
* Sensor-based robot control
* Basic embedded system design
* Motor driving logic
* Infrared sensor calibration
* I2C communication
* Multi-sensor integration
* MPU acceleration and gyroscope data reading
* Debugging through real-world testing
* Team cooperation and project documentation

This project also helped us realize that a robot program cannot only work in theory. It must be tested repeatedly in the real environment, because motor performance, sensor noise, ground material, battery voltage, wiring, and sensor placement can all influence the final result.

---

## Team Members

| Name          | Group    | Role                                                |
| ------------- | -------- | --------------------------------------------------- |
| Ruihang Liu   | Group 32 | Programming, testing, documentation                 |
| Xintong Jiang | Group 32 | Programming, hardware setup, testing, documentation |

---

## Purpose of This Repository

This GitHub repository is created for:

* Recording our project progress
* Sharing our code and testing experience
* Summarizing problems and solutions
* Helping future students understand similar robotics tasks
* Preserving the final results of Group 32

This repository is mainly for learning and communication purposes.
