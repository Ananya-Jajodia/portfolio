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
## Full Circuit Diagram
<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab4/circuit.png?raw=true" alt="Circuit Diagram">

I assembled the bot as shown below. I placed the IMU on a flat service and secured it by drilling holes and putting in zipties (idea from Shao Stassen). The ToF is secured with orange tape in this photo, but will be secured with a piece of double sided tape in the near future. Since it is important the ToFs are facing straight, I made sure to secure them onto a flat surface. The Artemis faces with the bluetooth antenna facing outward in the hope of minizing EMI interference with the bluetooth. The motor drivers are placed on either wall on either side of the Artemis, keeping the wires short. I also expanded the hole that the battery wire comes through to store the Artemis battery with the motor battery on the other side of the bot.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab4/fullbot.jpg?raw=true" alt="front of bot">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab4/battery.jpg?raw=true" alt="back of bot">



## Motor drivers
I started by soldering by motor drivers.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab4/driver.jpg?raw=true" alt="drivers">


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


## Testing Drive
I started with testing each side which connected to the power supply.

<iframe width="560" height="315" src="https://www.youtube.com/embed/f37mfaf5aR8?si=TH3ohDlebnH7ATaE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


Then, I connected the VINs and Grounds together and ran both motors.

<iframe width="560" height="315" src="https://www.youtube.com/embed/LQV6dQjj1qs?si=wT2d_kkCFDHV9FGb" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>




## Lowest PWM
To find the lowest PWM that Meep will move from rest with, I started with `loop` code that would try to drive forward at a set PWM for 3 seconds, then increase the pulse width by one. The robot started moving in the 35-39 range. I tested individual pulse widths from there till the robot began to move at 37 or 14.5% duty cycle.

<iframe width="560" height="315" src="https://www.youtube.com/embed/pcQM31gQiIw?si=DIib4T4HYDoaEYlQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



``` cpp
void loop() {
  forward(pwm); // Or left(pwm) depedning on which we want to control

  // SEND ANYTHING ACROSS THE SERIAL PORT TO STOP THE BOT
  if (Serial.available() > 0){
    a = Serial.readString();
    Serial.println("COMMAND TO WAIT SENT");
    wait = 1;
    while(wait){
      if (!Serial.available() > 0){
        // Serial.println("Waiting");
        stop();
      }
      
      // RESTART WITH NEW PWM WHEN SENT A NEW MESSAGE
      else {
        Serial.println("Reading");
        pwm = Serial.parseInt();
        wait = 0;
        Serial.print("PWM IS NOW: ");
        Serial.println(pwm);
      }
    }
    Serial.println("COMMAND TO CONTINUE SENT");
    while (Serial.available() > 0) {
      Serial.read(); // Clear Serial Input so we dont end up waiting 
    }
  }
}

```

For turning, the robot started moving at 93 with very jittery motion. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/2bX9u2Y0lmQ?si=619XJE3qEgkO604S" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


We see that the turning is more smooth starting at 100. It is good to note that turning requires a higher duty cycle than simple forward motion since there is more friction working against turning the wheels than just moving forward/backward in the direction the wheels are already facing.

<iframe width="560" height="315" src="https://www.youtube.com/embed/iCVf-zk2qtM?si=iCSEKqKTQdxeUZBo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


## Tuning the straight drive
To test how straight my robot drove, I ran both motors with 60 (23.5% duty cycle) for 6ft. Without calibration, Meep naturally drifted to the left.
<iframe width="560" height="315" src="https://www.youtube.com/embed/3EyGt7pgEsI?si=JrrDrEPofZPkGDvX" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


To fix this, I simply slowly lowered the right pwm so that the robot would stay on course. This fixed the straight drive as demonstrated in the video below.

```python
ble.send_command(CMD.FORWARD, "55|60")
```

```cpp
  case FORWARD: {
      int flspeed, frspeed;

      success = robot_cmd.get_next_value(flspeed);
      if (!success)
          return;

      success = robot_cmd.get_next_value(frspeed);
      if (!success)
          return;
          
      analogWrite(AB1_left,0); 
      analogWrite(AB2_left,flspeed);
      analogWrite(AB2_right,0);
      analogWrite(AB1_right,frspeed); 
      
      // Added so I can record the straight drive without the robot running away
      delay(4000);
      analogWrite(AB1_left,0); 
      analogWrite(AB2_left,0);
      analogWrite(AB1_right,0); 
      analogWrite(AB2_right,0);
      //////////////////////////////////////
      
      break;
  }
```


<iframe width="560" height="315" src="https://www.youtube.com/embed/WYSVoonZU9g?si=g9BWhASX9xjyva_Z" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


## Open Loop Robot Control
I ran open loop control with the robot to try to get it to head towards a wall and turn before it hit it. I did this with manual motion commands. This would be much easier to do with closed loop control. Meep would be able to use the time of flight sensor to tell when to turn and use the IMU to determine how much to turn by. We will be exploring this concept in the next lap.

<iframe width="560" height="315" src="https://www.youtube.com/embed/fnag7o3hDXg?si=b-ic6Hl4Io8iNKzl" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



## 5000: Analog Write
We can see from the PWM oscilloscope reading that period of the pwm signal is 5.460 ms. Using this, we can calculate the frquency to be 1/0.005460 or 183.15 Hz.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab4/freq.png?raw=true" alt="analog write">

```cpp
void forward(int ls, int rs) {
  analogWrite(AB1_left,ls); 
  analogWrite(AB2_left,0);
  analogWrite(AB1_right,0); 
  analogWrite(AB2_right,rs);
}

void loop(){
  for (i = 1; i <5; i++){
    forward(255/i, 255/i);
    delay(1000);
  }
}
```


## 5000: Lowest PWM with Motion
I found the lowest PWM that kept the bot in motion by lowering the PWM one by one until the bot stopped movement. The bot stopped moving at 44 which is odd since this is higher than the pwm needed to get it to move from rest. Upon rerunning my moving the vehicle from rest, I got a similar result as prior. I would expect this PWM to be lower since it takes less force to keep the bot in motion when it already has inertia in the direction we are trying to move. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/G7wxtmHMfKQ?si=-oRyOgfZi98tkney" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>




## Collaboration and Sources
I collobrated with Dyllan and Shao on bot building ideas. I referenced [Aiden Derocher's](https://boltstrike.github.io/pages/lab4.html) and [Tyler Wisniewski's](https://tylerwisniewski.github.io/lab4/) lab reports when building my robot.