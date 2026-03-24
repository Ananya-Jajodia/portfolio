+++
title = "Lab 7: Kalman Filter"
description = "This lab introduces the Kalman Filter"
date = "2026-03-23"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "PID", "PWM", "Time of Flight", "Kalman Filter"]
+++

# Lab 7: Kalman Filter
The goal of this lab is to implement a Kalman filter so the robot, Meep, is able to drive at a high speed towards a wall and stop before hitting it. We do this by creating a physics model to predict Meep's movement, then tuning a Kalman filter and using its predictions to estimate position while waiting for feedback from the time of flight.

## What is a Kalman Filter?
A Kalman filter is a way to combine a model for a system and sensor measurements to get an accurate estimate for the system. In this case, we have a time of flight that give accurate, but slow estimates for our position. By creating a physics model for our system, we can combine a calculated position with our sensor measurements to get quicker and more accurate location data for our robot. 

## Estimating Drag and Momentum
To start, we need to have an accurate physical model for our bot. Let `u` be the control signal (aka the PWM value or duty cycle) passed in to our motors), and let `x` be Meep's position, represented as distance from the wall infront of Meep. This leaves us with the equation below where `d` is a constant representing drag while `m` represents momentum.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/p1.png?raw=true" alt="physics">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/p2.png?raw=true" alt="digram">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/p3.png?raw=true" alt="state space equations">

We can find `d` by finding the steady speed for some `u` (motor contorl input). I chose to operate my car at an analog value of 150 (duty cycle of 58.82%). I ran 3 trials and calculated the velocity from each as shown below.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/triple.png?raw=true" alt="velocity">

From there, I took the average of the three runs and attempted to fit a expoential decay function to the measurements.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/vavg.png?raw=true" alt="velocity">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/vfitted.png?raw=true" alt="fitted velocity">

From here, we see the steady state velocity is -3176 mm/s. We can calculate `d` to be 150/the steady state velocity so that the control input can be the passed in PWM value in the future. We calculate `m` to be t_90/ln(1-0.9) where t_90 is the time it takes to get to 90% of the steady velocity. We have a rise time of 1.73 s giving us an `m` of 0.75.


```python
d = -0.047227600035556115
m = 0.7508005698913823
A = np.array([
    [0, 1],
    [0, d/m]
])

B = np.array([
    [0],
    [1/m]
])

C = np.array([[1,0]])

def kf(mu,sigma,u,y, dt, update = True):
    Ad = np.eye(len(A)) + dt * A  
    Bd = dt * B
    
    mu_p = Ad.dot(mu) + Bd.dot(u) 
    sigma_p = Ad.dot(sigma.dot(Ad.transpose())) + sig_u

    if update:
        sigma_m = C.dot(sigma_p.dot(C.transpose())) + sig_z
        kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))
    
        y_m = y-C.dot(mu_p)
        mu = mu_p + kkf_gain.dot(y_m)    
        sigma=(np.eye(2)-kkf_gain.dot(C)).dot(sigma_p)
        return mu,sigma
    else:
        return mu_p, sigma_p

```




## Tuning noise on simulated data
From here, we need to tune our uncertainity of our sensors and model. We have 4 parameters we can tune, the position and speed uncertainity from the physics model, the sensor noise, and the initial sigma. 

I started with a position and velocity noise of 100 and a sensor noise of 400. This noise was too large the sensor uncertainity. There was a major lag in the Kalman filter as sensor readings were bascially not trusted at all

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/big_noise.png?raw=true" alt="laggy kalman">


From there, I lowered the sensor noise as was able to see the filter follow the time of flight. I wanted the filter to be able to deviate from the time of flight so I chose a standard deviation of 10. This lead to the filter following the sensor almost exactly on all my datasets.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/good2.png?raw=true" alt="good kalman">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/good3.png?raw=true" alt="good kalman">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/good4.png?raw=true" alt="good kalman">

I then had my kalman extrapolate predicted positions for the time steps where the time of flight was not ready with a new reading. This resulted in the spiky graph below. 

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/spiky.png?raw=true" alt="bad extrapolating kalman">

This made me reevalute my position and velocity uncertainity. I expect the physics model to be relavtively accurate for the position but not for the velocity. I updated the sigmas to reflect this (decreaing position to 10 which increasing the velcoity uncertainity), to get the graph below:

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/extra.png?raw=true" alt="good extrapolating kalman">

```python
sigma_1 = np.sqrt((1**2)*1/dt) # position variance
sigma_2 =  np.sqrt((50**2)*1/dt) # speed variance
sigma_3 = 10 # sensor noise

sig_u=np.array([[sigma_1**2,0],[0,sigma_2**2]])
sig_z=np.array([[sigma_3**2]])
```


## Implement the Kalman Filter with PID
From there, I implemented the Kalman filter on the Artemis as shown below

```cpp
Matrix<2,1> mu = {0,0}; 
Matrix<2,2> sigma = {100,0,100,0}; 

float d = -0.047227600035556115;
float m = 0.7508005698913823;


Matrix<2,2> kA = {0, 1,
                 0, d/m}; 

Matrix<2,1> kB = {0, 1/m}; 

Matrix<1,2> kC = {1, 0}; 

float var_position = 9.09;
float var_speed = 227272.73;
float var_sensor = 100;

Matrix<2,2> sigma_u = {var_position, 0,
                        0, var_speed};

Matrix<1> sigma_z = {var_sensor};

float kf(float control_in, float sensor_reading, float dt, int tof_valid){
  Matrix<1> control = {control_in};
  Matrix<1> sensor = {sensor_reading};
  // Matrix<1> dt = {dt/1000.0};
  Matrix<2, 2> I = { 1, 0, 0, 1};
  Matrix<2, 2> Ad = I;
  Ad(0,0) += kA(0,0) * (dt);
  Ad(0,1) += kA(0,1) * (dt);
  Ad(1,0) += kA(1,0) * (dt);
  Ad(1,1) += kA(1,1) * (dt);
  Matrix<2, 1> Bd = kB;
  Bd(0,0) *= (dt);
  Bd(1,0) *= (dt);

  Matrix<2, 1> mu_p = Ad*mu + Bd*control;
  Matrix<2, 2> sigma_p = Ad*sigma*~Ad + sigma_u;

  if (tof_valid) {
    Matrix<1> sigma_m = kC * sigma_p * ~kC + sigma_z;
    Matrix<2,1> kff_gain = sigma_p*(~kC*Inverse(sigma_m));

    Matrix<1> y_m = sensor-kC*mu_p;
    mu = mu_p+kff_gain*y_m; 
    sigma = (I - kff_gain*kC)*sigma_p;

    return mu(0,0);
  }
  else {
    mu = mu_p;
    sigma = sigma_p;
    return mu_p(0,0);
  }

}
```

Then I integrated it with my pid code:


``` cpp
void pid_wall_dist(float dist){
  float curr_dist;
  time_stamps[i] = millis();
  if (!distanceSensor2.checkForDataReady()) {
    tof2_readings[i] = -5;

    // no sensor data on the first time step
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

    // no valid readings yet
    else if (tof2_readings[i-1] == -5 && tof1_readings[i-1] == -5){
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


    // Use kalman to predict position
    else {
      int dt = time_stamps[i] - time_stamps[i-1];

      if (dt == 0) {
        curr_dist = tof1_readings[i-1];   
      }
      else {
        curr_dist = kf(motor_data[i-1], 0, (float) dt/1000.0, false);
      }
      tof1_readings[i] = curr_dist;
    }
  }


  // Use Kalman with TOF to predict position and update the filter
  else {
    int dt = time_stamps[i] - time_stamps[i-1];
    tof2_readings[i] = distanceSensor2.getDistance();
    tof1_readings[i] = kf(motor_data[i-1], tof2_readings[i], (float) dt/1000.0, true);
    curr_dist =  tof1_readings[i];
    distanceSensor2.clearInterrupt();
    old_time = time_stamps[i];
  }
  float e = tof1_readings[i] - dist;
  error_total += e;

  // Integrator Windup code ///
  if (error_total > 2000){
    error_total = 2000;
  }
  else if (error_total < -3000){
    error_total = -3000;
  }
  /////////end//////////////

  error_change = e-last_error;
  int motor_out = (int)(Kp*e + Ki*error_total + Kd*error_change);

  p_data[i] = Kp*e;
  i_data[i] = Ki*error_total;
  d_data[i] = Kd*error_change;


  ///// Motor Updates //////////////
  if (motor_out > -8 && motor_out < 8){
    forward(0, 0);
    motor_out = 0; 
  }
  else if (motor_out > 5){
    if (motor_out < 37){
      motor_out = 37; 
    }
    else if (motor_out > 255){
      motor_out = 255;
    }
    forward(motor_out-3, motor_out+3);   
  }
  else {
    if (motor_out > -37){
      motor_out = -37; 
    }
    else if (motor_out < -255){
      motor_out = -255;
    }
    backward(-motor_out, -motor_out);
  }
  motor_data[i] = motor_out;
  ///////////end/////////////////
  
  // Step forward
  last_error = e;
  i++;
  if (i >= TIME_ARR_SIZE){
    i = 0;
  }
}
```


I started with only allowing the robot to run for 1 second before stopping it to ensure that any initial implementation bugs could be worked out without damage being done to Meep. Once, the Kalman filter was showing expected results, I was ready to run the full PID.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/short_irl.png?raw=true" alt="tof and kalman filter readings">


Using my original PID settings, we can see that Meep is able to start at its max duty cycle and stop just before hitting the wall.

<iframe width="560" height="315" src="https://www.youtube.com/embed/Ag1ghdkBwZI?si=nbDzXbmU6ujXHM1-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/irl_pid.png?raw=true" alt="tof and kalman filter readings">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab7/irl_motor.png?raw=true" alt="motor readings">





## Collaboration and Sources
I referenced the lecture slides when setting up the Kalman filter. I also referenced Trevor Dales, Aidan Derocher