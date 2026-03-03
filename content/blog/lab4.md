+++
title = "Lab 4: Motors and Open Loop Control"
description = "This lab introduces the motor driver and open loop control"
date = "2026-03-02"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "Open Loop", "PWM"]
+++

# Lab 4: Motors and Open Loop Control
In this lab, we work on controlling the robot, Meep, with open-loop pwm signals. We start by soldering the motor driver and testing the pwm signal generated via `analogWrite`. Meep has two motors and operates on differential drive.

# Wire Set Up
## Motor drivers
I decided to connect the motor drivers to A14, A15, A2, and A3. I did this for serval reasons:

1) I wanted each motor drivers to be on analog pins since that would be the most compatiable with the analogWrite command used to generate the PWM signals.

2) We needed to choose pins that can generate a pwm signal. By looking at the datasheet, we can see that pins with a tilde (~) can generate pwm signals. 

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab4/artemis.png?raw=true" alt="artemis pinout">

3) Since the wires to connect to the motors are on opposite sides of the bot, I chose pins that were on opposites sides of the Artemis. I wanted the wires to avoid crossing eachother and be as short as possible to reduce possible electromagnetic interference (EMI).


# PWM and Oscilloscope
## Power Supply Settings
I set my power supply to 3.70 V since I planned to use the 850 mAH lipo which has a nominal voltage of 3.7 V. I set the max current to 2 A so that the driver could run without tripping the power supply. After soldering the motor drivers to the Artemis, I generated a PWM signal with `analogWrite` and probed them as shown below. 

``` cpp
  #define AB2_left 2
  #define AB1_left 3
  #define AB2_right 14
  #define AB1_right 15
  Serial.begin(115200);
  pinMode(AB1_left,OUTPUT);
  pinMode(AB2_left,OUTPUT);
  pinMode(AB1_right,OUTPUT);
  pinMode(AB2_right,OUTPUT);
  analogWrite(AB1_left,0); 
  analogWrite(AB2_left,60);
  analogWrite(AB1_right,60); 
  analogWrite(AB2_right,0);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/_sbd9B9nU3s?si=-qcZMJTqlJsA015p" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

PWM, or pulse width modulation, is frequently used to control motor speeds. The PWM has a duty cycle, or a percentage of time when the signal is set to high. By lowering the duty cycle, the average voltage that the motor sees decreases, causing the motors to have a lower rpm. An example of the readings for 25%, 33%, 50%, and 100% duty cycle are shown below.

<iframe width="560" height="315" src="https://www.youtube.com/embed/dMtZA-7GfjU?si=9wTEe8oBbYrEGAU-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



## Lowest PWM
To find the lowest PWM that Meep will move from rest with, I started with `loop` code that would try to drive forward at a set PWM for 3 seconds, then increase the pulse width by one. The robot started moving in the 35-39 range. I tested individual pulse widths from there till the robot began to move at 37 or 14.5% duty cycle.

<iframe width="560" height="315" src="https://www.youtube.com/embed/pcQM31gQiIw?si=DIib4T4HYDoaEYlQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


# Open Loop Robot Control
I ran open loop control with the robot to try to get it to head towards a wall and turn before it hit it. I did this with manual motion commands. This would be much easier to do with closed loop control. Meep would be able to use the time of flight sensor to tell when to turn and use the IMU to determine how much to turn by. We will be exploring this concept in the next lap.

TODO: add video




## 5000: Analog Write
TODO: Do this




## 5000: Lowest PWM

TODO: DO THIS

## Collaboration and Sources
I collobrated with Dyllan and Shao on bot building ideas. I referenced [Aiden Derocher](https://boltstrike.github.io/pages/lab4.html) and [Tyler Wisniewski](https://tylerwisniewski.github.io/lab4/)