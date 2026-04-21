+++
title = "Lab 10: Grid Localization using Bayes Filter"
description = "In this lab, we execute a flip and drift using PID and Kalman filters"
date = "2026-04-20"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "Bayes Filter"]
+++

# Lab 10: Grid Localization using Bayes Filter
In this lab, we implement a bayes filter and ran it on a simulation of the environment from Lab 9. We do this to estimate our position in th given environment based on our sensor data and kinematic model.

## Implementing the filter

### Computing Control
First, we make a function to calculate the control infromation based on our current position and our previous position. Our control information is described in 3 parts. First, we have a rotation matrix that rotates the robot to face in the direction we wish to move. Then, we have a translation matrix that described the movement itself. Finally, we rotate so the robot is facing towards its cuurent yaw.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab10/control.png?raw=true" alt="equations and visual for controls from Helbling Lecuture">

```python
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    delta_rot_1 = mapper.normalize_angle(np.degrees(np.atan2(cur_pose[1] - prev_pose[1], cur_pose[0] - prev_pose[0])) - prev_pose[2])
    delta_trans = np.sqrt((cur_pose[0] - prev_pose[0])**2 + (cur_pose[1] - prev_pose[1])**2)
    delta_rot_2 = mapper.normalize_angle(cur_pose[2] - prev_pose[2] - delta_rot_1)
    return delta_rot_1, delta_trans, delta_rot_2
```

### Odometry Motion Model
Now that we can calulate the control, we can calculate the probability of that the control sequence occured given our current position, previous position, and the inputed control data.

```python
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)


    Returns:
        prob [float]: Probability p(x'|x, u)
    """
    
    u_pred = compute_control(cur_pose, prev_pose)

    prob_rot1 = loc.gaussian(u_pred[0], u[0], loc.odom_rot_sigma)
    prob_trans = loc.gaussian(u_pred[1], u[1], loc.odom_trans_sigma)
    prob_rot2 = loc.gaussian(u_pred[2], u[2], loc.odom_rot_sigma)

    return prob_rot1*prob_trans*prob_rot2
```

### Prediction Step
The next step is to update the belief model (bel_bar) based on control information from the previous timestep. We calculate the control information using the `compute_control` function we made earlier based on the preivous odometry and current odometry. From there, we loop through all posible discretized positions and orientations, calculate the probability of having moved from one to the other and use it to update the belief model. We skip states that we have a probability of less than 0.0001 to save computation time since it is incredible unlikely we are in these states anyway. 

```python
def prediction_step(cur_odom, prev_odom):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """

    u = compute_control(cur_odom, prev_odom)

    for cx in range(mapper.MAX_CELLS_X):
        for cy in range(mapper.MAX_CELLS_Y):
            for a in range(mapper.MAX_CELLS_A):
                if(loc.bel[cx,cy,a]) >= 0.0001:
                    for cx2 in range(mapper.MAX_CELLS_X):
                        for cy2 in range(mapper.MAX_CELLS_Y):
                            for a2 in range(mapper.MAX_CELLS_A):        
                                prev_pose = mapper.from_map(cx, cy, a)
                                cur_pose = mapper.from_map(cx2, cy2, a2)
                                loc.bel_bar[cx2,cy2,a2] += odom_motion_model(cur_pose, prev_pose, u)*loc.bel[cx,cy,a]
```

### Sensor Model
To update the sensor model, we are given the true observations for a robot pose on the map. We loop through the range measurements made by the robot by doing a 360 degree turn in place while taking `OBS_PER_CELL` equally spaced distance measurements. We then calculate the probability of seeing each of those measurements based on the true observations and the sensor noise.

``` python 
def sensor_model(obs):
    """ This is the equivalent of p(z|x).


    Args:
        obs ([ndarray]): A 1D array consisting of the true observations for a specific robot pose in the map 

    Returns:
        [ndarray]: Returns a 1D array of size 18 (=loc.OBS_PER_CELL) with the likelihoods of each individual sensor measurement
    """

    prob_array = np.zeros(mapper.OBS_PER_CELL)
    for i in range(mapper.OBS_PER_CELL):

        prob_array[i] = loc.gaussian(loc.obs_range_data[i][0], obs[i], loc.sensor_sigma)


    return prob_array
```
### Update Step
Finally, we update the belief with our sensor model information. For each cell, the probability that we are there is the probability we are there based on the sensor model times the probability we are there based on our control information. We calculate this value for each cell, then normalize these probabilities so they add to 1.


```python
def update_step():
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """

    for cx in range(mapper.MAX_CELLS_X):
        for cy in range(mapper.MAX_CELLS_Y):
            for a in range(mapper.MAX_CELLS_A):
                sensor_prob = np.prod(sensor_model(mapper.get_views(cx, cy, a)))
                loc.bel[cx, cy, a] = sensor_prob * loc.bel_bar[cx, cy, a]
    loc.bel = loc.bel/np.sum(loc.bel)
```

## Results
This is the graph of the robot running with the bayes filter. The green line shows the real location of the robot. We see that the model is least accurate when the robot is turning for the first time and at the end when it is entering the large area on the left. This is likely due to the robot entering areas where the walls are far away. This makes the sensor less reliable and gives it less distinguisable readings to help it narrow down its location.


### Run A
<iframe width="560" height="315" src="https://www.youtube.com/embed/04Dq2jt2Ki0?si=LLKrkQxYHAfiyQXy" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Run B
<iframe width="560" height="315" src="https://www.youtube.com/embed/JQJrF0ggmbU?si=hQeTOBpGWkvqlaFf" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


## Collaboration and Sources
I refferenced [Lucca Correia](https://correial.github.io/LuccaFastRobots/Fast%20Robots%20Stuff/lab-10/)'s and [Aidan Derocher](https://boltstrike.github.io/pages/lab10.html)'s Lab 10 handouts for debugging help. I also reference lecture slides [18](https://fastrobotscornell.github.io/FastRobots-2026/lectures/FastRobots2026_Lecture18_motionmodels.pdf) and [19](https://fastrobotscornell.github.io/FastRobots-2026/lectures/FastRobots2026_Lecture19_sensormodels.pdf) from Cornell's 2026 Fast Robots class.
