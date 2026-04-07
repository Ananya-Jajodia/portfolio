+++
title = "Lab 8: Stunts"
description = "This lab introduces the Kalman Filter"
date = "2026-04-06"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "PID", "PWM", "Time of Flight", "Kalman Filter", "IMU"]
+++

# Lab 8: Stunts
In this lab, our goal is make the robot car do a stunt. I started by making the car do a flip, but switched to the drift when I lost access to the sticky pads.


## Implementing the flip
For the flip, stunt is implemeneted in 4 parts.

First, we drive towards the wall infront of us. Next, we execute the flip. Then, we correct the angle of the car. Finally, we drive back to the start.

To code this, I decided to separate it out into stages as highlighted above:


For the first stage, I used the Kalman filter from the previous lab to get accurate distance data as Meep sped towards the wall.
``` c
    case 0: {
      // get start angle so we know what angle we need to target when heading back
      if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
      }
      while(!((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))){
        if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
          myICM.readDMPdataFromFIFO(&imu_data);
        }
      }
      
      get_roll_pitch_yaw(0);
      curr_setpoint = yaw_readings[0] - 180;
      if (curr_setpoint > 180){
        curr_setpoint -= 360;
      }
      else if (curr_setpoint< -180) {
        curr_setpoint += 360;
      }
      
      while (!distanceSensor2.checkForDataReady()){}
      mu(0,0) = distanceSensor2.getDistance();
      distanceSensor2.clearInterrupt();
      sigma(0,0) = 100;
      sigma(1,1) = 100;
      starting_point = mu(0,0);
      tof1_readings[0] = starting_point;
      tof2_readings[0] = starting_point;
      motor_data[0] = 150;
      float curr_dist = starting_point;
      time_stamps[0] = millis();
      forward(speed-3, speed+3);
      i = 1;



      // Head towards the wall
      while(mu(0,0) > dist){
        time_stamps[i] = millis();
        float dt = (time_stamps[i]-time_stamps[i-1])/1000.0;
        if (dt > 0){
          if(!distanceSensor2.checkForDataReady()){
            curr_dist = kf(speed, 0, dt, false);
            tof1_readings[i] = -5;
          }
          else {
            tof1_readings[i] = distanceSensor2.getDistance();
            curr_dist = kf(speed, tof1_readings[i], dt, true);
            distanceSensor2.clearInterrupt();
          }
          tof2_readings[i] = curr_dist;
          motor_data[i] = 150;
          i++;
          if (i >= TIME_ARR_SIZE){
            i = 0;
          } 
        }
      }

      // When we read the desired distance, execute the flip
      stage = 1; 
      break;
    }
```

Next, we execute the flip itself by moving full speed backwards.

``` c
    // flip (gun it backwards)
    case 1: {
      backward(speed, speed);
      delay(800);
      stop();
      int cont_time = millis() + 800;
      while (millis() < cont_time){
        if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
          myICM.readDMPdataFromFIFO(&imu_data);
        }
      }
      stage = 2;
      break;
```

Finally, we angle correct using the PID controller we made in lab 6.

``` c
// angle correct
case 2: {
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
    myICM.readDMPdataFromFIFO(&imu_data);
    }
    while(!((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))){
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
    }
    }
    get_roll_pitch_yaw(i-1);
    setpoint_angle[i-1] = curr_setpoint;
    float e = yaw_readings[i-1] - curr_setpoint;
    if (e > 180){
    e -= 360;
    }
    else if (e < -180) {
    e += 360;
    }

    while (e > 3 || e < -3 ){
    tof1_readings[i] = -10;
    tof2_readings[i] = -10;
    pid_angle(curr_setpoint);
    e = yaw_readings[i-1] - curr_setpoint;
    if (e > 180){
        e -= 360;
    }
    else if (e < -180) {
        e += 360;
    }
    }
    stage = 3;
    break;
}
```



Then, we move forward to return to start.


## Drift
To execute the drift, we remove stage 1 (the actual flipping) and just execute the angle correction. 


We start with the prestunt. We can see that the Kalman Filter allows us to predict our position quickly and accurately.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab8/preflip.png?raw=true" alt="tof and kalman filter readings">


Next, we drift to 180 degrees

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab8/imu.png?raw=true" alt="tof and kalman filter readings">

At this point, I had the issue of bot overshooting the turn. I implemented a wait stage so Meep would make sure it was at the correct angle before progressing.

``` c
case 3: {
    brake();
    delay(5);
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
    }
    while(!((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))){
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&imu_data);
    }
    }
    get_roll_pitch_yaw(i);
    setpoint_angle[i] = curr_setpoint;
    float e = yaw_readings[i] - curr_setpoint;
    if (e > 180){
        e -= 360;
    }
    else if (e < -180) {
        e += 360;
    }
    
    // MAKE SURE THE ANGLE IS CORRECT
    if (e > 3 || e < -3 ) {
        stage = 2; // BACK TO PID IF ITS BAD
    }

    // PROGRESS IF ITS GOOD
    else {
        stage = 4;
    }
    break;
}
```


Here are the results. I adjusted the Kp, Ki, and Kd as I went. I found that a lower Kp actually caused more overshoot as shown in the first video.

<iframe width="560" height="315" src="https://www.youtube.com/embed/YIAmRXd7Vak?si=_fMXIT7k4R1n5k5I" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


<iframe width="560" height="315" src="https://www.youtube.com/embed/V6829iFoS18?si=g8Hn38cv9botZC4U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


<iframe width="560" height="315" src="https://www.youtube.com/embed/cnvtcc6dPug?si=zzuR-kYFlJjTvXQT" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Blooper

On my way to making the flip work, I ended up executing both stunts as shown in the video below:

<iframe width="560" height="315" src="https://www.youtube.com/embed/oBGvP337xO4?si=WhtS1QSsjvi8OtG_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Collaboration and Sources
None. Thanks to the TAs for the help!