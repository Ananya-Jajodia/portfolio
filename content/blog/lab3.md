+++
title = "Lab 3: ToF"
description = "This lab introduces the ToF sensor"
date = "2026-02-24"

[taxonomies]
tags = ["ece5160", "Artemis", "C Programming", "ToF"]
+++

# Lab 3: Time of Flight Sensors

This lab explores adding two time of flight sensors to an RC car (Meep). We explore how to handle I2C communication when devices share the same address, the pros and cons of different configuration modes, and the various sensitivities of infared sensors. 


## Time of Flight Setup

### Vehicle Planning
First, we need to chose where we will be placing our ToF sensors on the car (Meep). I chose to place one sensor in the front and one sensor on the right of the car. This will allow Meep to see if there is something in front of it and avoid a collision. At the same time, having a sensor to the right gives potenital for Meep to follow a wall to help it determine if it is drifting in any direction. This also give for potential for maze naviagtion in the future. 

The disadvantages of this placement are the sensor will be on the back if the robot flips. For mapping our environment in the future, we would have to spin 270 degrees to get a complete mapping. Meep will also not be able to follow or see obstacles on its left.

TODO: make diagram of where stuff goes


### Soldering and initial testing

We start by soldering the ToF pins to a QWiic connect cable for easy use. As seen in the digram above, the 4 pins of the qwiic connect are soldered to their corresponding pins. Note that red is power, black is ground, blue is SDA (I2C data wire), and yellow is SCL (I2C clock wire).

TODO: add pic of soldering job

By running the Apollo Example05_wire_I2C demo code, we can see the I2C addresses of our devices. The I2C address is the memory location where sensor data will be written on our Artemis as determined by the sensor manufacturer. 

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/i2c_addr_tof.png?raw=true" alt="i2c address of tof and imu" />


We can see that the IMU is on 0x69 and that the ToF is on 0x029. An important note is that both ToFs are on the same I2C address by default. This means that the information from the time of flights will conflict and likely corrupt eachother if we allow both to try to write at the same time. We fix this by using the XSHUT pin and the `setI2CAddress` function. The XSHUT pin allows us to turn on and off the ToF sensors. By keeping one of the ToF sensors off, we can change the I2C address of the other sensor without conflict. We change the I2C address of the second sensor to 0x30 since we see that it is not in use. This allows us to have both sensors on the same I2C bus without conflict.

``` cpp
// Create Objects for both ToF sensors
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN);
SFEVL53L1X distanceSensor2;

pinMode(SHUTDOWN_PIN, OUTPUT); 
digitalWrite(SHUTDOWN_PIN, LOW); // shut down the sensor so we can change its I2C address
distanceSensor1.setI2CAddress(0x30); // Set an address that won't conflict
digitalWrite(SHUTDOWN_PIN, HIGH); // reboot sensor with new address
```

TODO: add pic of demo code running with both sensors

<!-- * location placement and soldering job
* I2C address
* running the demo code with one
* using the xshut -->


## Distance Modes, Accuracy, and Execution speed

Next, we explore the different distance modes of the ToF sensor. The ToF sensor has three different distance modes: short, medium, and long. Each mode has different accuracy and execution speed. I speficially chose to compare the short and long distance modes. The short distance mode has a range of 0.1m to 1.2m and an accuracy of +/- 1mm. The long distance mode has a range of 0.1m to 4m and an accuracy of +/- 3mm. The short distance mode has an execution time of 20ms while the long distance mode has an execution time of 50ms. 

I experiemented with this by setting one sensor to short distance mode and the other sensor to long distance mode. I then moved the sensors backwards slowly and recorded the distance data from both sensors.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/tof_distance_diff.png?raw=true" alt="long vs short distance modes" />

As shown, the sensors are very accurate in their respective ranges. The short distance mode is more accurate at close distances while the long distance mode is more accurate at longer distances. As soon as the range is exceeded, the sensors become noisy before spitting out erroneous data. This could be a problem in the future since these erroneous data look like valid data so Meep might think it is close to or colliding with an object when it is not. 

<!-- * Picture of your ToF sensor connected to your QWIIC breakout board
* Screenshot of Artemis scanning for I2C device (and discussion on I2C address)
* Discussion and pictures of sensor data with chosen mode
* 2 ToF sensors and the IMU: Discussion and screenshot/video of sensors working in parallel
* Tof sensor speed: Discussion on speed and limiting factor; include code snippet of how you do this
* Time v Distance: Include graph of data sent over bluetooth (2 sensors)
* Time v Angle: Include graph of data sent over bluetooth -->

## 5000: IMU Color and Texture Sensitivity


## 5000: Other Infared Sensors
