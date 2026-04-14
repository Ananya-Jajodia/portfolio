+++
title = "Lab 9: Stunts"
description = "In this lab, we execute a flip and drift using PID and Kalman filters"
date = "2026-04-14"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "PID", "Mapping", "Time of Flight", "IMU"]
+++

# Lab 9: Mapping
The goal of this lab is to use the IMU and time of flight sensors on Meep to create a map of the environment. We sample distances from multiple points in the area, use transformation matricies to translate the measurements from the robot frame to the world frame, then convert this scan to a Line-Based Map that can be used in the simulation for lab 10.

## Orientation Control Mapping

### Implementation
To get a full map of the environment, we get a 360 degrees of distance measurements from set points in the environment. We then compile these readings together to make a map. In order to get the 360 degree mapping, Meep must read distance values while spinning in place. I decided to do this using the orientation control I tuned in [lab 6](https://ananya-jajodia.github.io/portfolio/blog/lab6/). I created the command `MAP` which allows to caller to determine how many datapoints they want from the turn. Meep will calculate the setpoint angles based on the number of readings desired.

``` c
case MAP: {
  success = robot_cmd.get_next_value(x_coord);
  success = robot_cmd.get_next_value(y_coord);
  success = robot_cmd.get_next_value(num_angles);
  success = robot_cmd.get_next_value(Kp);
  success = robot_cmd.get_next_value(Ki);
  success = robot_cmd.get_next_value(Kd);
  
  angle_step = 360/num_angles;
  angle_idx = 0;
  i = 0;
  mapping = true;
}
```


From there, we simply record and transmit mapping data as shown below:


``` c
void make_mapping(){
  
  // We start by getting a yaw value
  if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
    myICM.readDMPdataFromFIFO(&imu_data);
  }
  while(!((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))){
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(&imu_data);
    }
  }
  get_roll_pitch_yaw(0);

  // We set the first yaw value as our first setpoint
  curr_setpoint = yaw_readings[0];
  // and record values from both our time of flights
  record_tof_vals(num_datapoints, curr_setpoint);
  

  for (int j = 0; j < num_angles; j++){
    // we head to the next angle
    curr_setpoint += angle_step;
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
    
    // while we aren't at the desired angle, we call `pid_angle`
    while (e > 5 || e < -5 ) {
      pid_angle(curr_setpoint);
      e = yaw_readings[i-1] - curr_setpoint;
      if (e > 180){
        e -= 360;
      }
      else if (e < -180) {
        e += 360;
      }
    }
    
    // once we get there, we stop and record the time of flight and angle data
    brake();
    tx_estring_value.clear();
    tx_estring_value.append("GETTING DATA");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());  
    record_tof_vals(num_datapoints, curr_setpoint);  
  }


  ////////////////// Start Transmition ////////////////////////////

  // Once we finish, we send the data over
  tx_estring_value.clear();
  tx_estring_value.append("Transmitting");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  

  for (int j = 0; j < angle_idx; j++){
    tx_estring_value.clear();
    tx_estring_value.append(time_stamps[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(tof1_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(tof2_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(gyro_yaw_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(setpoint_angle[j]);
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
```

When actually recording the data, I record `num_vals` datapoints to try to offset the error and noise discussed below.

``` c
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



### Testing
To test this, I created a box for Meep to try mapping 


<iframe width="560" height="315" src="https://www.youtube.com/embed/tPS64UNq5q4?si=L3s2kEL690oNum_K" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


That resulted in the map below.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab9/box.png?raw=true" alt="tof and kalman filter readings">

Since I noticed the side time of flight having some much smaller readings, I adjusted its angle. That resulted in the graph below

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab9/fixed.png?raw=true" alt="tof and kalman filter readings">


My sensor readings mostly agree. The discrepency between them is likely the result of having different distances due to their physical placement. As the car rotates, the front sensor will typically get closer to the wall it is reading from than the side sensor. Even in this small space, we already see quite a bit of error in both the sensors. I would reason that, in an empty room, its likely that one of the sensors would hallunicate some sort of wall or blockage that does not exist. It is likely that we are going to get a lot of noise data points and will have to look for where walls appear on mulitple trials to confirm where the real walls are located.

### Real Environment

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab9/irl_env.HEIC?raw=true" alt="tof and kalman filter readings">

This is the environment we are trying to map. I measured the distance at 20 different angles, taking 5 measurements at each to try to offset bad readings. I got data from several locations in the environment that were a set distance apart. Specifically, I grabed data from (0,0), (-3,-2), (5,3), (0,3), and (5,-3). At each point, Meep would rotate and gather data.

<iframe width="560" height="315" src="https://www.youtube.com/embed/uGFTZiANPq4?si=xdePEET0Mam9ahS_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The mapping from each point can easily be seen in polar coordinates as shown below.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab9/polar.png?raw=true" alt="tof and kalman filter readings">




## Collaboration and Sources
I consulted Jack Long's website and discussed rotation vector mapping with Dyllan Hofflich.