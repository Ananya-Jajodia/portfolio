+++
title = "Lab 11: Localization on the real robot"
description = "In this lab, we use a bayes filter for localization using a real robot"
date = "2026-04-27"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "Bayes Filter"]
+++

# Lab 11: Localization on the real robot
In this lab, we implement a bayes filter and run in in the environment from lab 9 to estimate the robot's position on our sensor data.

## Simulation
We start by running a simulation to test localization. Once we confirm this is working, we can move on to implementing localization on the real robot.


Odometry is in red, Ground truth in green, and belief in blue.
<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab11/sim.png?raw=true" alt="lab 11 sim screenshot">


## Reality
To do localization in reality, we need to be able to collect equidistant points in a circle. We can use the code that we used to get sensor readings in `Lab 9`:

``` cpp
void make_mapping(){
  if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
    myICM.readDMPdataFromFIFO(&imu_data);
  }
  while(!((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))){
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(&imu_data);
    }
  }
  get_roll_pitch_yaw(0);
  curr_setpoint = yaw_readings[0];
  record_tof_vals(num_datapoints, curr_setpoint);
  for (int j = 0; j < num_angles; j++){
    curr_setpoint -= angle_step;
    if (curr_setpoint > 180){
      curr_setpoint -= 360;
    }
    else if (curr_setpoint < -180) {
      curr_setpoint += 360;
    }
    float e = yaw_readings[i-1] - curr_setpoint;
    if (e > 180){
      e -= 360;
    }
    else if (e < -180) {
      e += 360;
    }
    // Make sure we're at the desired angle
    tx_estring_value.clear();
    tx_estring_value.append("GOING TO SETPOINT ");
    tx_estring_value.append(curr_setpoint);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());  
    while (e > 3 || e < -3 ) {
      pid_angle(curr_setpoint);
      e = yaw_readings[i-1] - curr_setpoint;
      if (e > 180){
        e -= 360;
      }
      else if (e < -180) {
        e += 360;
    }
    }
    brake();
    tx_estring_value.clear();
    tx_estring_value.append("GETTING DATA");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());  
    record_tof_vals(num_datapoints, curr_setpoint);  
  }

  tx_estring_value.clear();
  tx_estring_value.append("Transmitting");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  
  
  ////////////////// Start Transmition ////////////////////////////

  // Once we finish, we send the data over
  for (int j = 0; j < angle_idx; j++){
    tx_estring_value.clear();
    tx_estring_value.append(tof2_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(gyro_yaw_readings[j]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(&imu_data);
    }
  }
  tx_estring_value.clear();
  tx_estring_value.append("Done");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());          
  Serial.println("Done");
  mapping = false;
}

// Records Time of Flight values. In this case, we only need 1 value per angle
void record_tof_vals(int num_vals, int angle){
  for (int j = 0; j < num_vals; j++){
    while (!distanceSensor2.checkForDataReady()) {
      if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
      }
    }
    while (!distanceSensor1.checkForDataReady()) {
      if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
      }
    }
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(&imu_data);
    }
    while(!((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))){
      if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
      }
    }
    time_stamps[angle_idx] = millis();
    get_roll_pitch_yaw(angle_idx);
    gyro_yaw_readings[angle_idx] = yaw_readings[angle_idx];
    tof1_readings[angle_idx] = distanceSensor1.getDistance();
    tof2_readings[angle_idx] = distanceSensor2.getDistance();
    setpoint_angle[angle_idx] = angle;

    distanceSensor1.clearInterrupt();
    distanceSensor2.clearInterrupt();
    angle_idx++;
    if (angle_idx >= TIME_ARR_SIZE){
      angle_idx = 0;
    }
  }
}
```

We program our `perform_observation_loop` to call this code with the `MAP` command.

```python
    async def perform_observation_loop(self, rot_vel=120):
        """Perform the observation loop behavior on the real robot, where the robot does  
        a 360 degree turn in place while collecting equidistant (in the angular space) sensor
        readings, with the first sensor reading taken at the robot's current heading. 
        The number of sensor readings depends on "observations_count"(=18) defined in world.yaml.
        
        Keyword arguments:
            rot_vel -- (Optional) Angular Velocity for loop (degrees/second)
                        Do not remove this parameter from the function definition, even if you don't use it.
        Returns:
            sensor_ranges   -- A column numpy array of the range values (meters)
            sensor_bearings -- A column numpy array of the bearings at which the sensor readings were taken (degrees)
                               The bearing values are not used in the Localization module, so you may return a empty numpy array
        """
        sensor_ranges = []
        sensor_bearings = []

        def get_sensor_circle(uuid, byte_message):
            mess = ble.bytearray_to_string(byte_message)
            sep = mess.find(":")
            
            if sep != -1: 
                try: 
                    sensor_ranges.append(float(mess[:sep])/1000.0)
                    sensor_bearings.append(float(mess[sep+1:]))
                except:
                    print(f"Something went wrong with: {mess}")
            else: 
                print(mess)
                
        async def sleep_for_30_secs():
            # Data collection takes around 15 seconds, but we wait 30 to ensure we do not end before it is ready
            await asyncio.sleep(30) 
        
        ble.start_notify(ble.uuid['RX_STRING'], get_sensor_circle)
        ble.send_command(CMD.MAP, "18|1.8|0.01|0")
        await sleep_for_30_secs()
        ble.stop_notify(ble.uuid['RX_STRING'])                     
        return np.array(sensor_ranges)[np.newaxis].T, np.array(sensor_bearings)[np.newaxis].T
```

From there, we can observe the robot's estimated position at various locations in the map.


<iframe width="560" height="315" src="https://www.youtube.com/embed/tQhlqhKmUXQ?si=yNtETx9ZfI8_OIyO" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

At (0,0) and (-3,-2), the belief and truth were entirely overlapping. Both (0,0) and (-3,-2) have obvious features close by that allow the robot to guess its location more successfully. Unique wall configurations are close by which helps the location of the bot to be narrowed down.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab11/exact00.png?raw=true" alt="blue dot overlaps green">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab11/exact-3-2.png?raw=true" alt="blue dot overlaps green">

For the rest of the points, the values were slightly off. 

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab11/oof03.png?raw=true" alt="blue dot and green dot">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab11/oof53.png?raw=true" alt="blue dot and green dot">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab11/oof5-3.png?raw=true" alt="blue dot and green dot">


<table border="1" cellpadding="6" cellspacing="0">
  <thead>
    <tr>
      <th>Coordinate</th>
      <th>Ground Truth</th>
      <th>Estimated Position</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>(0, 0)</td>
      <td>(0.0, 0.0, 0)</td>
      <td>(0.000, 0.000, -10.000)</td>
    </tr>
    <tr>
      <td>(-3, -2)</td>
      <td>(-0.9144, -0.6096, 0)</td>
      <td>(-0.914, -0.610, -10.000)</td>
    </tr>
    <tr>
      <td>(5, 3)</td>
      <td>(1.524, 0.914, 0)</td>
      <td>(1.524, 0.610, -10.000)</td>
    </tr>
    <tr>
      <td>(5, -3)</td>
      <td>(1.524, -0.914, 0)</td>
      <td>(1.829, -1.219, -10.000)</td>
    </tr>
    <tr>
      <td>(0, 3)</td>
      <td>(0, 0.914, 0)</td>
      <td>(-0.305, 0.914, -10.000)</td>
    </tr>
  </tbody>
</table>


All the angles were off by 10 degrees. This may be the result of the angle I placed the bot at not being exactly 0. The time of flight might also be slightly angled on the bot.

The positions being slightly off could be the result of the sensor not rotating exactly on the dot since it is offset from the center of the car. This could also be due to less precise features helping narrow down the position or just general sensor noise.



<!-- (0.000, 0.000, -10.000) [0, 0]  0.0881415
(-0.914, -0.610, -10.000) [-3, -2] 0.1015790
(1.524, 0.610, -10.000) [5, 3] 0.1912395
(1.219, 0.610, -10.000) [5, -3]     
(1.829, -1.219, -10.000) [5, -3] 0.3210059
(-0.305, 0.914, -10.000) [0, 3] 0.9067523 -->


## Collaboration and Sources
I referred to [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab11/)'s lab report when writing mine.