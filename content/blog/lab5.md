+++
title = "Lab 5: Linear PID control and Linear Interpolation"
description = "This lab introduces PID control"
date = "2026-03-09"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "PID", "PWM"]
+++

# Lab 5: Linear PID control and Linear interpolation
The goal of this lab was to get the robot, Meep, to drive forward and stop extactly 304 mm from a wall via PID controls. This lab explores tuning a PID controller, extrapolating sensor data, and handling integrator windup.



## Why PID?
PID or Proportional, Inetegral, and Derivative controller is a controller that adjusts the PWM signal based on the bots current error from a set point. In this lab, the setpoint is the distance away from the wall and the error is determined to be the difference between the car's estimated position (via Time of Flight) and the setpoint. 

I chose to implement a full PID controller for this lab. I started with just a proportional term, which resulted in either heavy overshoot or stopping a great distance from the target. While I added the I term (required since I am a grad student), this overshoot issue got worse. To correct for this, the derivative term was added. I implemented a `PID_START` and `PID_STOP` command to signal to the robot to start and stop the PID cotnroller. The `PID_START` command takes in values for `Kp`, `Ki`, and `Kd` and triggers the `pid_wall_dist` function to run as shown below:

``` c
case PID_START: {
  Serial.println("Starting PID");
  success = robot_cmd.get_next_value(Kp);
  success = robot_cmd.get_next_value(Ki);
  success = robot_cmd.get_next_value(Kd);
  if (!success)
      return;

  pid_running = true; //tell loop to call pid_wall_dist
  
  // reset variables
  i = 0;
  error_total = 0; 
  last_error = 0;
  error_change = 0;
  break;
}
```




``` c
// Takes in [dist] which is the setpoint in mm
void pid_wall_dist(float dist){
  float curr_dist;
  time_stamps[i] = millis();
  
  // if there is no sensor reading
  if (!distanceSensor2.checkForDataReady()) {
    tof2_readings[i] = -5;

    // no data to extrapolate with
    if (i == 0) {
      tof1_readings[i] = -5;
      p_data[i] = 0;
      i_data[i] = 0;
      d_data[i] = 0;
      motor_data[i] = 0;
      i++;
      if (i >= TIME_ARR_SIZE){
        i = 0;
      }
      return ;
    }

    // only one reading or no valid readings
    else if (i == 1 || tof1_readings[i-1] == -5 || tof1_readings[i-2] == -5){
      tof1_readings[i] = tof1_readings[i-1];
      p_data[i] = p_data[i-1];
      i_data[i] = i_data[i-1];
      d_data[i] = d_data[i-1];
      motor_data[i] = motor_data[i-1];
      i++;
      if (i >= TIME_ARR_SIZE){
        i = 0;
      }
      return ;
    }

    // Can extrapolate next tof value
    else {
      int dt = time_stamps[i-1] - time_stamps[i-2];

      if (dt == 0) {
        curr_dist = tof1_readings[i-1];   // fallback
      }
      else {
        curr_dist = (old_slope) *                    //  m
              (time_stamps[i] - time_stamps[i-1]) +  // *x
              tof1_readings[i-1];                    // +b
      }
      if (curr_dist < 0) {
        tof1_readings[i] = dist;
      }
      else if (curr_dist > 5000){
        tof1_readings[i] = 4000;
      }
      else {
        tof1_readings[i] = curr_dist;
      }
    }
  }

  // if there is a sensor reading
  else {
    tof1_readings[i] = distanceSensor2.getDistance();
    tof2_readings[i] = tof1_readings[i];
    curr_dist =  tof1_readings[i];
    distanceSensor2.clearInterrupt();
    old_slope = (curr_dist - old_tof)/(time_stamps[i]-old_time);
    old_tof = curr_dist;
    old_time = time_stamps[i];
  }

  // update errors and controller
  float e = curr_dist - dist;
  error_total += e;
  error_change = e-last_error;
  int motor_out = (int)(Kp*e + Ki*error_total + Kd*error_change);

  p_data[i] = Kp*e;
  i_data[i] = Ki*error_total;
  d_data[i] = Kd*error_change;
  motor_data[i] = motor_out;

  // update motors (corrected for straight drive)
  if (motor_out > 0){
    forward(motor_out-3, motor_out+3);   
  }
  else {
    backward(-motor_out, -motor_out);
  }
  last_error = e;
  
  i++;
  if (i >= TIME_ARR_SIZE){
    i = 0;
  }
}
```


## Prelab: Data transfer
One of the major decisions of this lab way how to tranfer data. Transferring each piece of data in the control loop 
would slow the loop down, resulting in slower updates and slower reaction time. Trying to store all the data is limited by the maximum storage of the Artemis. I decide to store all my values and transmit after PID has stopped. This results in a long waiting time after running my controller, but allows the code to loop fasting during the original run time. I allows the buffer to override the inital values if the controller tries to save more values than the set max buffer length. I parse this data in python lists like I have in previous labs and save them to a CSV for the future.


``` c
case PID_STOP: {
  // Stop PID controller and the motors
  pid_running = false;
  analogWrite(AB1_left,0); 
  analogWrite(AB2_left,0);
  analogWrite(AB1_right,0); 
  analogWrite(AB2_right,0);

  // tranfer the data written
  for (int j = 0; j < i; j++){
    tx_estring_value.clear();
    tx_estring_value.append(time_stamps[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(tof1_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(tof2_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(p_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(i_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(d_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(motor_data[j]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
  }
  
  // Signal the data tranfer is done and it is safe to stop the listener
  tx_estring_value.clear();
  tx_estring_value.append("Done");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());          
  break;
}
```



## Range/Sampling time
I chose to swap my time of flight from short range mode to long range so that the max distance would be 4m instead of 1.5m. I originally decided to use short range mode since it takes far less time to be ready with a distance reading. However, we work on extrapolating values so that we can update the motors with an educated guess of our distance readings. This prevents the polling delay on the time of flight from being a limiting factor in the speed between motor updates. Since the delay becomes a non-issue, the benefits of long range mode outweigh the cons.


With the original loop waiting for the time of flight to be ready, the average time between motor updates was **100.29 ms**. I modified the loop to skip when the time of flight was not ready. This sped the loop up to and average of **2.4 ms** between updates. Iterations that required processing a time of flight value took much longer, while iterations with no new time of flight reading typically ran very fast. Finally, when data extrapolation was added (as dsicussed below) the loop ran updating the motor values every **7.68 ms** on average. 



### Data Extrapolation
We use linear extrapolation to estimate the next time of flight value when one isn't available. The way I decided to do this was to use the previous two valid time of flight readings to make a slope. I orginally used the last two readings to calcualte the slope (including the last two estimated readings). This was problematic since the estimated readings were often smaller than the real distaces since the bot tends to slow down as it approaches the wall. When an accurate ToF reading came in, the slope would become postive and the code would assume Meep was moving away from the wall rather than towards it. To find the next value, we simply use slope-intercept with the change in time and previous estimated distance.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab5/tof_extrapolation.png?raw=true" alt="PID TOF graph">


```c
// Code to calculate slope when a new distance reading is available
tof1_readings[i] = distanceSensor2.getDistance();
tof2_readings[i] = tof1_readings[i];
curr_dist =  tof1_readings[i];
distanceSensor2.clearInterrupt();
old_slope = (curr_dist - old_tof)/(time_stamps[i]-old_time);
old_tof = curr_dist;
old_time = time_stamps[i];
```


``` c
// using point slope to estimate when data isn't available
curr_dist = (old_slope) *                    //  m
      (time_stamps[i] - time_stamps[i-1]) +  // *x
      tof1_readings[i-1];                    // +b

// Sanity checks to make sure we aren't making up a crazy distance 
if (curr_dist < 0) {
  tof1_readings[i] = 0;
}
else if (curr_dist > 5000){
  tof1_readings[i] = 4000;
}
else {
  tof1_readings[i] = curr_dist;
}
```

## Tuning process and results
I started with just a proportional controller. Since my max distance from the time of flight was 4000 mm and the setpoint was 304 mm, the max error was This resulted in some successfully runs, but the controller tended to overshoot and hit the wall and have steady state error in both the negative and positive direction. Here's a video of the most successful run on the gain only controller.

<iframe width="560" height="315" src="https://youtube.com/embed/Een-ilGlYeE?si=ARzaI3TNV_qQCyFA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



From there, I decided to add the integral term, since I wanted to see if PID was needed or if PI would work well. I was reminded that Ki should typically be a very small value as the cummlative error can get to a large number fairly quickly.

<iframe width="560" height="315" src="https://www.youtube.com/embed/EasqB4KuETA?si=14vEp_7DYyQApHkX" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


Since I was experiencing a large amount of overshoot, I decided to add the derivative term and use a full PID controller. I restarted my tuning process, finding a Kp that resulted in the car being stable with steady state error (0.06). I added a small Ki to try to correct for this (0.0001). At this point, the car began to overshoot its set point, so I added Kd (1). I was able to successfully stop at the set point in these conditions.

<iframe width="560" height="315" src="https://www.youtube.com/embed/ubvJBLO58g8?si=n5wruhzsdi6t5vW8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab5/tof_data.png?raw=true" alt="TOF graph">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab5/motor_data.png?raw=true" alt="PID graph">



The PID controller should also be able to find the wall again when displaced. I pulled the car back to show that it was able to recorrect.

<iframe width="560" height="315" src="https://www.youtube.com/embed/VZQbFTPQEzI?si=-CymJpBajm6gOitm" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


## Grad Task: Integrator Windup
As previously mentioned, the cummalitive error can get very large very quickly, especially when first starting the controller. Integrator windup occurs when the integrator term gets overpopulated very quickly when the setpoint is set. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/doazuYY04rw?si=00tKecE6dtijwDO6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

We can avoid this by limiting the max value of the total error. This prevents the error accumulation from blowing up, even if we keep the bot away from the set point for an extended period of time.

``` c
if (error_total > 2000){
  error_total = 2000;
}
else if (error_total < -3000){
  error_total = -3000;
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/GoN54xzSpCY?si=MncTcP97lxBsJ0fI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


## Collaboration and Sources
I referenced the lecture slides when setting up PID. I discussed ideas for code structure with Dyllan Hofflich.