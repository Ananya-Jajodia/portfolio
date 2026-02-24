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

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/car.png?raw=true" alt="car" />

### Soldering and initial testing

We start by soldering the ToF pins to a QWiic connect cable for easy use. As seen in the digram above, the 4 pins of the qwiic connect are soldered to their corresponding pins. Note that red is power, black is ground, blue is SDA (I2C data wire), and yellow is SCL (I2C clock wire).

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/solder.jpg?raw=true" alt="soldering job" />
<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/battery.jpg?raw=true" alt="battery" />

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

<iframe width="560" height="315" src="https://www.youtube.com/embed/SCR4AWbkjnM?aTTMHf2pFS0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


<!-- * location placement and soldering job
* I2C address
* running the demo code with one
* using the xshut -->


For futher testing, I taped both ToFs to a box so that they would be in the same position and facing the same direction. This way, we can compare the data from both sensors to see if they are consistent with eachother. 


<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/box.jpg?raw=true" alt="box with both sensors" />


## Distance Modes
Next, we explore the different distance modes of the ToF sensor. The ToF sensor has three different distance modes: short, medium, and long. Each mode has different accuracy and execution speed. I speficially chose to compare the short and long distance modes. The short distance mode has a range of 0.1m to 1.2m and an accuracy of +/- 1mm. The long distance mode has a range of 0.1m to 4m and an accuracy of +/- 3mm. The short distance mode has an execution time of 20ms while the long distance mode has an execution time of 50ms. 

I experiemented with this by setting one sensor to short distance mode and the other sensor to long distance mode. I then moved the sensors backwards slowly and recorded the distance data from both sensors.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/tof_distance_diff.png?raw=true" alt="long vs short distance modes" />

As shown, the sensors are very accurate in their respective ranges. The short distance mode is more accurate at close distances while the long distance mode is more accurate at longer distances. As soon as the range is exceeded, the sensors become noisy before spitting out erroneous data. This could be a problem in the future since these erroneous data look like valid data so Meep might think it is close to or colliding with an object when it is not. 


The long distance mode is also much slower than the short distance mode. This would be a problem for Meep since it needs to be able to react quickly to its environment.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/tof_speed_diff.png?raw=true" alt="long vs short distance modes" />

With this in mind, we expect Meep to only need to see objects that are close to it in order to avoid collisions. Meep can also drive close to walls for mapping in the future. With this in mind, I will be using the short distance mode for both sensors.


## Accuracy
Using both sensors, we can determine how accurate the sensors are in short distance mode. I measured the distance from the sensor to a wall using a tape measure and compared it to the distance reported by the sensor. I did this for distances of 0.2m, 0.5m, 0.75m, 1m, and 1.51m. We can see that the sensors output different values for the same distance. This is likely due to a difference in facing angle of the sensor to the wall. Overall, we see the sensors are very accurate with a maximum percent error of 8.57% for sensor 2 at 0.2m. Sensor 1 is more accurate than sensor 2 at all distances. This could be due to a difference in the quality of the sensors or a difference in the facing angle of the sensors to the wall.

<table border="1" cellpadding="6" cellspacing="0">
  <thead>
    <tr>
      <th>Real Value (mm)</th>
      <th>Sensor 1 (mm)</th>
      <th>Sensor 2 (mm)</th>
      <th>Difference</th>
      <th>Percent Error 1</th>
      <th>Percent Error 2</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>200.0</td>
      <td>193.51</td>
      <td>217.14</td>
      <td>-23.63</td>
      <td>-0.0325</td>
      <td>0.0857</td>
    </tr>
    <tr>
      <td>500.0</td>
      <td>494.00</td>
      <td>522.64</td>
      <td>-28.64</td>
      <td>-0.0120</td>
      <td>0.04528</td>
    </tr>
    <tr>
      <td>750.0</td>
      <td>747.37</td>
      <td>772.05</td>
      <td>-24.68</td>
      <td>-0.0035</td>
      <td>0.0294</td>
    </tr>
    <tr>
      <td>1000.0</td>
      <td>994.37</td>
      <td>1024.96</td>
      <td>-30.59</td>
      <td>-0.0056</td>
      <td>0.02496</td>
    </tr>
    <tr>
      <td>1510.0</td>
      <td>1517.00</td>
      <td>1548.50</td>
      <td>-31.50</td>
      <td>0.0046</td>
      <td>0.0255</td>
    </tr>
  </tbody>
</table>



<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/distance_setup.jpg?raw=true" alt="distance measurement setup" />
<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/distance_box.jpg?raw=true" alt="distance measurement setup" />


## Execution Time

Next, I attempted to see how quickly I could get data from one of the sensors to print to the Serial Monitor. I decided to have the sensor print data as fast as possible and using the built in time stamps on the serial monitor to determine the execution time so that recoding the time would not slow it down. The average time between serial prints was 3.87 ms.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/serial.png?raw=true" alt="serial monitor times" />


``` cpp
void loop(void)
{
  if (distanceSensor1.checkForDataReady())
  {
    distance = distanceSensor1.getDistance();

    Serial.print("TOF1: ");
    Serial.print(distance);
    Serial.println();
  }
}
```

## Integration with IMU and Bluetooth

Finally, I integrated the ToF sensors with the IMU and Bluetooth. I created the command `SENSOR_START` for the Artemis to begin saving and the transmitting the data from the sensors. I then used the `SENSOR_STOP` command to stop transmitting data. I demonstrate this by sldiing a box with the sensors taped on back and forth in front of a wall. The data is read and saved by a notification handler (see previous labs for more details on this).

The average time bewtween data points was 6.41 ms.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/tof_reading.png?raw=true" alt="tof readings" />

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/imu.png?raw=true" alt="imu readings" />


<!-- * Picture of your ToF sensor connected to your QWIIC breakout board
✓ Screenshot of Artemis scanning for I2C device (and discussion on I2C address)
* Discussion and pictures of sensor data with chosen mode
* 2 ToF sensors and the IMU: Discussion and screenshot/video of sensors working in parallel
✓ Tof sensor speed: Discussion on speed and limiting factor; include code snippet of how you do this
✓ Time v Distance: Include graph of data sent over bluetooth (2 sensors)
✓ Time v Angle: Include graph of data sent over bluetooth -->

## 5000: IMU Color and Texture Sensitivity
I tested the IMU's sensitivity to different colors and textures by placing different materials in front of the IMU at 200 mm and taking the average of the measureuments for a set period of time. I found that the IMU was sensitive to different materials but not overly so. I noticed the most discrepancy with the glass door which was expected since the most light passes through it. Suprisingly, the black garbage bag and the black expomarker also performed poorly. This could indicate a sensitivity to black or a sensitivity to the texture of the materials.

<table border="1" cellpadding="6" cellspacing="0">
  <thead>
    <tr>
      <th>Real Value (mm)</th>
      <th>Surface</th>
      <th>Sensor 1 (mm)</th>
      <th>Sensor 2 (mm)</th>
      <th>Difference</th>
      <th>Percent Error 1</th>
      <th>Percent Error 2</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/door.jpg?raw=true" alt="door" />
      White door</td>
      <td>193.51</td>
      <td>217.14</td>
      <td>-23.63</td>
      <td>-0.03245</td>
      <td>0.0857</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/paper.jpg?raw=true" alt="paper" />Paper</td>
      <td>184.36</td>
      <td>200.53</td>
      <td>-16.17</td>
      <td>-0.0782</td>
      <td>0.00265</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/wall.jpg?raw=true" alt="white wall" />White Wall</td>
      <td>193.46</td>
      <td>207.47</td>
      <td>-14.01</td>
      <td>-0.0327</td>
      <td>0.03735</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/whiteboard.jpg?raw=true" alt="whiteboard" />Whiteboard</td>
      <td>178.97</td>
      <td>194.64</td>
      <td>-15.67</td>
      <td>-0.10515</td>
      <td>-0.0268</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/marker.jpg?raw=true" alt="expomarker" />Expomarker</td>
      <td>167.77</td>
      <td>191.2</td>
      <td>-23.43</td>
      <td>-0.16115</td>
      <td>-0.044</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/tape.jpg?raw=true" alt="orange tape" />Orange Tape</td>
      <td>178.89</td>
      <td>197.8</td>
      <td>-18.91</td>
      <td>-0.10555</td>
      <td>-0.011</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/cloth.jpg?raw=true" alt="green microfiber" />Green Microfiber</td>
      <td>183.18</td>
      <td>203.14</td>
      <td>-19.96</td>
      <td>-0.0841</td>
      <td>0.0157</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/jacket.jpg?raw=true" alt="pink polyester" />Pink Polyester</td>
      <td>175.82</td>
      <td>199.84</td>
      <td>-24.02</td>
      <td>-0.1209</td>
      <td>-0.0008</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/door.jpg?raw=true" alt="glass door" />Glass Door</td>
      <td>118.57</td>
      <td>188.69</td>
      <td>-70.12</td>
      <td>-0.40715</td>
      <td>-0.05655</td>
    </tr>
    <tr>
      <td>200.0</td>
      <td><img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab3/garbage.jpg?raw=true" alt="garbage bag" />Garbage Bag</td>
      <td>169.3</td>
      <td>190.86</td>
      <td>-21.56</td>
      <td>-0.1535</td>
      <td>-0.0457</td>
    </tr>
  </tbody>
</table>

## 5000: Other Infared Sensors

There are two main types of infared distance sensors. The first is like the time of flight we are using, which measures distance based on the time it takes for a light signal to be recieved back after it is emitted. The second type is a triangulation sensor, which measures distance based on the angle of the reflected light signal. The triangulation sensor is more accurate at close distances but time of flight sensors have a longer range. Time of flights also tend to be faster than triangulation sensors. This makes time of flights better for applications where you need to see objects that are farther away or need to react quickly to your environment. Triangulation sensors seem to be best for when you need a very precise measurement of distance at close range.

## Collaboration and Sources
I worked with Shao Stassen and Dyllan Hofflich when soldering the ToF sensors. I worked with Shao when gathering data for accuracy. Aiden Derocher's lab 3 report was also referenced.

[Short vs Long Distance Mode information](https://www.pololu.com/product-info-merged/3415)

[Infared Sensor Types](https://meskernel.net/en/tof-vs-triangulation/)