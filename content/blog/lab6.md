+++
title = "Lab 6: Orientation Control
"
description = "This lab introduces PID control for orientation"
date = "2026-03-09"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "PID", "PWM", "IMU"]
+++

# Lab 6: Orientation Control

The goal of this lab was to get the robot, Meep, to be able to turn to a specified orientation, and maintain that position when encountering disturbances. I implement and tune a PI controller that shows success and varying terain conditions. PID or Proportional, Inetegral, and Derivative controller is a controller that adjusts the PWM signal based on the bots current error from a set point. In this lab, the setpoint is a given orientation angle and the error is the difference between estimated angle (via IMU readings) and the setpoint. The proportion term is important for obvious reasons, I also implemented the integral term to help with steady error and make the controller more effective on varying floor conditions (as discusses below).

## Prelab: Data transfer
One of the major decisions of this lab way how to tranfer data. Transferring each piece of data in the control loop would slow the loop down, resulting in slower updates and slower reaction time. Trying to store all the data is limited by the maximum storage of the Artemis. I decide to store all my values and transmit after PID has stopped. This results in a long waiting time after running my controller, but allows the code to loop fasting during the original run time. I allow the buffer to override the inital values if the controller tries to save more values than the set max buffer length but set the buffer length to the largest number possible to prevent this case from occurring. I parse this data in python lists like I have in previous labs and save them to a CSV for the future.

```c
case PID_STOP: {
  // Stop PID and motors
  pid_running = false;
  analogWrite(AB1_left,0); 
  analogWrite(AB2_left,0);
  analogWrite(AB1_right,0); 
  analogWrite(AB2_right,0);

  // Transfer Data
  for (int j = 0; j < i; j++){
    tx_estring_value.clear();
    tx_estring_value.append(time_stamps[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(roll_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(pitch_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(yaw_readings[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(p_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(i_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(d_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(motor_data[j]);
    tx_estring_value.append(":");
    tx_estring_value.append(setpoint_angle[j]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // drain the DMP FIFO so the chip doesn't crash
    if (myICM.status = ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(&imu_data);
    }
  }

  // Signal that the data is done sending
  tx_estring_value.clear();
  tx_estring_value.append("Done");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());          
  break;

  Serial.println("Done");
}
```

I also added the `SET_ANGLE` command, which allows the user to change the setpoint while PID is running.  

``` c
case SET_ANGLE: {
    success = robot_cmd.get_next_value(curr_setpoint);
    if (!success)
        return;
    break;
}
```

## Range/Sampling time and DMP
I enabled the DMP (Digital Motion Processing) to get calibrated accurate IMU angles. The DMP fills a Queue (First In, First Out) with values from the desired report. I chose to get the Game Rotation Vector which return the quaterion of the angle facing, calcultated by combinding the accelerometer and gyroscope values. I chose to not use the magnetometer since Meep only needs to know its relative orientation and I wasn't sure how the megnetometer would perform in different environments. I was able to run its max frequency of about 55 Hz which resulted in an average of **17.6 ms** between motor updates.



## Tuning process and results
I started with just a proportional controller on a smooth but grippy floor. With proportion drive, Meep was able to quickly turn to the desire angle with minimal overshoot and negliable steady state error. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/mmdaHxOOke0?si=lzY8MEJxuMwXZc_U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

I adjusted the P term to be **1.9** and tested it on a set sequence. I had the robot reach 0, 90, -90, 45, and 0 degrees with 5 seconds between setpoint changes. I did this to test accuracy, speed, and ability to adjust to a changing setpoint. I saw very accurate and quick positioning. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/mEOyG15NwhU?si=vt0cEIjaYrrXUwKX" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


// TODO put p_motor_seq and angle_seq


Next, I tried running the robot on carpet and immediately ran into problems. The robot struggled to turn with the currently set deadband region.

<iframe width="560" height="315" src="https://www.youtube.com/embed/o-HdEpyZORU?si=kyGb1EnPoRhT9ndw" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

// TODO put carpet_fail here

Since I didn't want to adjust the deadband region but wanted Meep to be able to turn accurately on all floor types, I decided to add the integral term. An integral term of **0.1** allowed Meep to turn accurately on carpet.

<iframe width="560" height="315" src="https://www.youtube.com/embed/IpFZIKDALGo?si=fzSt-GIIcYkBRAOn" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

// TODO put angle_carpet

We can see the addition of the I term allowed Meep to get enough power to turn successfully!

// TODO pu motor_error_carpet

Finally, I tested Meep with its newly tuned PI controller on a tile floor, and gave him a few kicks to watch him readjust.

<iframe width="560" height="315" src="https://www.youtube.com/embed/ZqC2eZY8XSM?si=hnRcx0W4Kqk5zQXd" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

// TODO put angle_kick
// TODO put motor_error_kick
/ angle_pi_tile

## Grad Task: Integrator Term
The `i` term is proportional to the cummalative error. Without being capped, the error term can greatly accumulate causing a high amount of overshoot once the robot is unstuck. In this lab, this causes the robot to be stuck spinning since the error in unable to decrease so the PWM is always maxed out.

<iframe width="560" height="315" src="https://www.youtube.com/embed/_PO8hTbDdt8?si=L3qEGnAusd8wPUoQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

// TODO put windup


We can fix this by capping the max value. Earlier in the lab, I capped the cummlative error to 2000. I decided to decrement the cap to 1000 so that Meep would not be stuck overshooting (osciallating) for as long.

<iframe width="560" height="315" src="https://www.youtube.com/embed/f1d4OXWDE-8?si=WiruG1SxlO6HmOiq" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

// TODO put capped_windup


## Collaboration and Sources
I did the lab with very light discussion with Dyllan Hofflich.